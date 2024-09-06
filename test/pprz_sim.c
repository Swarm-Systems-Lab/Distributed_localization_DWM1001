#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <pthread.h>

typedef enum ss_packet_types
{
	SS_P_DEBUG			= 0x0,
	SS_P_IDENTITIES,
	SS_P_CONFIRMATION,
	SS_P_NED_POS,
	SS_P_SOURCE_DIST,
	SS_P_CENTROID,
	SS_P_ASC_DIR,
	SS_P_FIELD_MEASURE
} ss_packet_t;

typedef enum sd_read_state
{
	SD_SYNC0,
	SD_SYNC1,
	SD_CLASS,
	SD_LENGTH,
	SD_DATA
} sd_read_state_t;

typedef struct serial_packet
{
	uint8_t p_class;
	uint8_t p_length;
	uint8_t p_data[64];
} serial_packet_t;

typedef struct dev_data
{
	int fd;
	serial_packet_t uart1_recv_buff[1];
	uint16_t r_identifier_map[3];
} dev_data_t;

uint16_t identifier_map[3] = {18, 1955, 3213};

// Function to configure the serial port
int configure_port(int fd) {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        return -1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag &= ~IGNBRK;         // Disable break processing
    tty.c_lflag = 0;                // No signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                // No remapping, no delays
    tty.c_cc[VMIN]  = 1;            // Read at least 1 byte
    tty.c_cc[VTIME] = 1;            // 0.1 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);        // Ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // Shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return -1;
    }
    return 0;
}

void process_message(dev_data_t* dev_data)
{
	printf("Message %u class %u len %u\n", dev_data->fd, (dev_data->uart1_recv_buff)[0].p_class, (dev_data->uart1_recv_buff)[0].p_length);
	switch ((dev_data->uart1_recv_buff)[0].p_class) 
	{
		case SS_P_CONFIRMATION :
			memcpy((dev_data->r_identifier_map), (dev_data->uart1_recv_buff)[0].p_data, sizeof(dev_data->r_identifier_map));
			printf("Confirmed %u [%u,%u,%u]\n", dev_data->fd, (dev_data->r_identifier_map)[0], (dev_data->r_identifier_map)[1], (dev_data->r_identifier_map)[2]);
			break;
		case SS_P_CENTROID :
			printf("Centroid %u (%f,%f)\n", dev_data->fd, ((float*)((dev_data->uart1_recv_buff)[0].p_data))[0], ((float*)((dev_data->uart1_recv_buff)[0].p_data))[1]);
			break;
		case SS_P_ASC_DIR :
			printf("Pos %u (%f,%f)\n", dev_data->fd, ((float*)((dev_data->uart1_recv_buff)[0].p_data))[0], ((float*)((dev_data->uart1_recv_buff)[0].p_data))[1]);
			break;
		default:
	}
}

// Thread function to read from a device
void* read_device(void* arg) {
    dev_data_t* dev_data = (dev_data_t*)arg;

    uint8_t current_byte = 0;
	size_t data_cnt = 0;
	sd_read_state_t state = SD_SYNC0;

	while(true)
	{
		int n = read(dev_data->fd, &current_byte, 1);

		if (n > 0)
		{
			// printf("%c\n", current_byte);
			switch (state)
			{
				case SD_SYNC0:
					data_cnt = 0;
					if (current_byte == 'D')
						state = SD_SYNC1;
					break;
				case SD_SYNC1:
					if (current_byte == 'W')
						state = SD_CLASS;
					else
						state = SD_SYNC0;
					break;
				case SD_CLASS:
					(dev_data->uart1_recv_buff)[0].p_class = current_byte;
					state = SD_LENGTH;
					break;
				case SD_LENGTH:
					(dev_data->uart1_recv_buff)[0].p_length = current_byte;
					state = SD_DATA;
					break;
				case SD_DATA:
					if (data_cnt < (dev_data->uart1_recv_buff)[0].p_length)
					{
						(dev_data->uart1_recv_buff)[0].p_data[data_cnt] = current_byte;
						data_cnt++;
					}
					else
					{
						process_message(dev_data);
						state = SD_SYNC0;
					}
					break;
			}
		}
	}

    return NULL;
}

void send_msg(int fd, uint8_t msg_class, uint8_t len, uint8_t *data)
{
	unsigned char command = 'D';
	
	// send header
	write(fd, &command, 1);
	command = 'W';
	write(fd, &command, 1);
	command = msg_class;
	write(fd, &command, 1);
	command = len;
	write(fd, &command, 1);

	// send payload
	for (uint8_t i = 0; i < len; i++)
	{
		command = data[i];
		write(fd, &command, 1);
	}
}

// Thread function to write to a device
void* write_device(void* arg) {
    dev_data_t* dev_data = (dev_data_t*)arg;

	while (true)
	{
		if (memcmp(identifier_map, dev_data->r_identifier_map, sizeof(identifier_map)) != 0)
		{
			printf("Send id %u\n", dev_data->fd);
			send_msg(dev_data->fd, SS_P_IDENTITIES, 6, (uint8_t*)identifier_map);
		}
		else 
		{
			float p[2] = {1, 1};
			switch (dev_data->fd)
			{
			case 4:
				p[0] = 2 + (rand()&0x1);
				p[1] = 2;
				break;
			case 5:
				p[0] = 3;
				p[1] = 3 + (rand()&0x1);
				break;
			default:
				break;
			}
			send_msg(dev_data->fd, SS_P_NED_POS, 8, (uint8_t*)p);
		}
	
		usleep(200000);
	}

    return NULL;
}

int main() {
    char* ports[] = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", NULL}; // List your serial ports
    pthread_t read_threads[10], write_threads[10];
 	dev_data_t* dev_data[10];
    int i = 0;

    while (ports[i]) {
		dev_data[i] = malloc(sizeof(dev_data[i]));
        dev_data[i]->fd = open(ports[i], O_RDWR | O_NOCTTY | O_SYNC);
        if (dev_data[i]->fd < 0) {
            perror("open");
            exit(EXIT_FAILURE);
        }

        if (configure_port(dev_data[i]->fd) != 0) {
            close(dev_data[i]->fd);
            exit(EXIT_FAILURE);
        }

		printf("Created for %u\n", dev_data[i]->fd);
        pthread_create(&read_threads[i], NULL, read_device, dev_data[i]);
        pthread_create(&write_threads[i], NULL, write_device, dev_data[i]);
        i++;
    }

    for (int j = 0; j < i; j++) {
        pthread_join(read_threads[j], NULL);
        pthread_join(write_threads[j], NULL);
    }

    for (int j = 0; j < i; j++) {
        close(dev_data[j]->fd);
    }

    return 0;
}
