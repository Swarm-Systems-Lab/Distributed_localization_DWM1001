#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <dirent.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <pthread.h>
#include <stdatomic.h>

#define MAX_LINE_LENGTH 1024
#define MAX_FILENAME_LENGTH 256
#define DEVICE_NUM		3

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
	int id;
	serial_packet_t uart1_recv_buff[1];
	uint16_t r_identifier_map[DEVICE_NUM];
} dev_data_t;

#pragma pack (1)
typedef struct pprz_gen_data
{
	float timestamp;
	float enu_pos[2];
	float sigma;
	float centroid[2];
	float asc_dir[2];
} pprz_gen_data_t;

uint16_t identifier_map[DEVICE_NUM] = {18, 1955, 3213};

pprz_gen_data_t exp_data[DEVICE_NUM][3600];

// Index: [data_size]][device_n][spacial_dim]
float centroids[DEVICE_NUM][2];
float asc_dirs[DEVICE_NUM][2];

uint16_t entry_count = 0;
pthread_barrier_t barrier;

uint8_t write_thd_end[DEVICE_NUM] = {0,0,0};

uint16_t csv_entry_num = 50000;

// Function to configure the serial port
int configure_port(int fd)
{
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0)
	{
        perror("tcgetattr");
        return -1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag &= ~IGNBRK;         // Disable break processing
    tty.c_lflag = ~ICANON;          // No signaling chars, no echo, canonical processing
    tty.c_oflag = 0;                // No remapping, no delays
    tty.c_cc[VMIN]  = 0;            // Read at least 1 byte
    tty.c_cc[VTIME] = 10;            // 0.1 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);        // Ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // Shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
	{
        perror("tcsetattr");
        return -1;
    }
	
    return 0;
}

void process_message(dev_data_t* dev_data)
{
	// printf("Message %u class %u len %u\n", dev_data->id, (dev_data->uart1_recv_buff)[0].p_class, (dev_data->uart1_recv_buff)[0].p_length);
	switch ((dev_data->uart1_recv_buff)[0].p_class)
	{
		case SS_P_CONFIRMATION :
			memcpy((dev_data->r_identifier_map), (dev_data->uart1_recv_buff)[0].p_data, sizeof(dev_data->r_identifier_map));
			// printf("Confirmed %u [%u,%u,%u]\n", dev_data->id, (dev_data->r_identifier_map)[0], (dev_data->r_identifier_map)[1], (dev_data->r_identifier_map)[2]);
			break;
		case SS_P_CENTROID :
			// Very unlikely or impossible RAW Race cond with entry count
			memcpy(centroids[dev_data->id], (dev_data->uart1_recv_buff)[0].p_data, sizeof(centroids[dev_data->id]));
			// printf("C_%u_(%f,%f)\n", dev_data->id, ((float*)((dev_data->uart1_recv_buff)[0].p_data))[0], ((float*)((dev_data->uart1_recv_buff)[0].p_data))[1]);
			break;
		case SS_P_ASC_DIR :
			memcpy(asc_dirs[dev_data->id], (dev_data->uart1_recv_buff)[0].p_data, sizeof(asc_dirs[dev_data->id]));
			// printf("A_%u_(%f,%f)\n", dev_data->id, ((float*)((dev_data->uart1_recv_buff)[0].p_data))[0], ((float*)((dev_data->uart1_recv_buff)[0].p_data))[1]);
			break;
		case SS_P_SOURCE_DIST :
			// printf("Sigma_%u_(%f)\n", dev_data->id, *((float*)((dev_data->uart1_recv_buff)[0].p_data)));
			break;
		default:
	}
}

// Thread function to read from a device
void* read_device(void* arg) {
    dev_data_t* dev_data = (dev_data_t*)arg;

    uint8_t current_byte = 0;
	size_t data_cnt = 0;
	uint16_t message_count = 0;
	sd_read_state_t state = SD_SYNC0;

	while(message_count < 3000 && !write_thd_end[dev_data->id])
	{
		int n = read(dev_data->fd, &current_byte, 1);

		// printf("%c\n", current_byte);
		if (n > 0)
		{
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
						message_count++;
						state = SD_SYNC0;
					}
					break;
			}
		}
	}

	// printf("Exit read %u", dev_data->id);

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
	struct timespec t_start, t_end;

	while (entry_count < csv_entry_num)
	{
		if (memcmp(identifier_map, dev_data->r_identifier_map, sizeof(identifier_map)) != 0)
		{
			// printf("Send id %u\n", dev_data->id);
			send_msg(dev_data->fd, SS_P_IDENTITIES, 6, (uint8_t*)identifier_map);
		}
		else 
		{
			// Send GPS pos
			// Paparazzi send with 10hz
			clock_gettime(CLOCK_MONOTONIC, &t_end);
			double elapsed = (t_end.tv_sec - t_start.tv_sec) + (t_end.tv_nsec - t_start.tv_nsec) / 1e9;
			if (elapsed > 0.25)
			{
				t_start = t_end;
				send_msg(dev_data->fd, SS_P_NED_POS, 12, (uint8_t*)(exp_data[dev_data->id][entry_count].enu_pos));
				// printf("Send pos\n");
				pthread_barrier_wait(&barrier);
			}
		}
	
		usleep(10000);
	}

	write_thd_end[dev_data->id] = true;
	// printf("Exit write %u", dev_data->id);

    return NULL;
}

uint16_t read_csv(const char *filename, pprz_gen_data_t data[], int max_data_count)
{
    FILE *file = fopen(filename, "r");
	char line[MAX_LINE_LENGTH];
    uint16_t line_count = 0;
	uint16_t token_count = 0;
	uint16_t column_pos[3] = {0,0,0};
	char *token_p = NULL;
	pprz_gen_data_t entry;
	pprz_gen_data_t def_entry = {.timestamp=0.0, .enu_pos={0.0,0.0}, .sigma=0.0};

    if (file == NULL)
	{
        printf("Error: Could not open file %s\n", filename);
        return -1;
    }

    // Get CSV format
    fgets(line, MAX_LINE_LENGTH, file);
	token_p = strtok(line, "\t");
	
	while (token_p != NULL)
	{
		if (strncmp("Time", token_p, sizeof("Time")) == 0)
			column_pos[0] = token_count;
		if (strncmp("DWM1001_DATA:enu_pos", token_p, sizeof("DWM1001_DATA:enu_pos")) == 0)
			column_pos[1] = token_count;
		if (strncmp("DWM1001_DATA:sigma", token_p, sizeof("DWM1001_DATA:sigma")) == 0)
			column_pos[2] = token_count;

		token_p = strtok(NULL, "\t");
		token_count++;
	}

    while (fgets(line, MAX_LINE_LENGTH, file) && line_count < max_data_count)
	{
		// Tokenize the line
        token_p = strtok(line, "\t");
		token_count = 0;
		entry = def_entry;
		
		while (token_p != NULL)
		{
			if (token_count == column_pos[0])
				entry.timestamp = strtof(token_p, NULL);
			if (token_count == column_pos[1])
				sscanf(token_p, "%f,%f", &(entry.enu_pos[0]), &(entry.enu_pos[1]));
			if (token_count == column_pos[2])
				entry.sigma = strtof(token_p, NULL);

			token_p = strtok(NULL, "\t");
			token_count++;
		}

		data[line_count] = entry;
		line_count++;
    }

    fclose(file);
    return line_count;  // Return the number of entries read
}

int is_csv(const char *filename) 
{
    size_t len = strlen(filename);
    return len > 4 && strcmp(filename + len - 4, ".csv") == 0;
}

// Function to get the first 'n' .csv files from the directory
uint8_t get_csv_files(const char* folder_name, int n, char* filenames) 
{
    DIR *dir;
    struct dirent *entry;
    int count = 0;
    
    if ((dir = opendir(folder_name)) == NULL)
	{
        perror("Unable to open directory");
        return 0;
    }

    // Read directory contents
    while ((entry = readdir(dir)) != NULL && count < n)
	{
        // Check if the file has a .csv extension
        if (is_csv(entry->d_name))
		{
            snprintf(filenames+count*MAX_FILENAME_LENGTH, MAX_FILENAME_LENGTH, "%s", entry->d_name);
            count++;
        }
    }

    closedir(dir);
    
    return count;
}


int main() {
    char *ports[] = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", NULL}; // List your serial ports
    pthread_t read_threads[10], write_threads[10];
 	dev_data_t* dev_data[10];
    int i = 0;
    const char filename[DEVICE_NUM][MAX_FILENAME_LENGTH];
    
	uint8_t files_found = get_csv_files(".", DEVICE_NUM, (char*)filename);

    // Read the CSV file
	for (uint8_t j = 0; j < DEVICE_NUM; j++)
	{
   		uint16_t line_n = read_csv(filename[j], exp_data[j], 3600);
		if (line_n < csv_entry_num)
			csv_entry_num = line_n;
	}

	memset(centroids, 0, sizeof(centroids));
	memset(asc_dirs, 0, sizeof(asc_dirs));

	pthread_barrier_init(&barrier, NULL, 4);

    while (ports[i])
	{
		dev_data[i] = malloc(sizeof(dev_data_t));
        dev_data[i]->fd = open(ports[i], O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
		dev_data[i]->id = i;

        if (dev_data[i]->fd < 0)
		{
            perror("open");
            exit(EXIT_FAILURE);
        }

        if (configure_port(dev_data[i]->fd) != 0)
		{
            close(dev_data[i]->fd);
            exit(EXIT_FAILURE);
        }

		// printf("Created for %u\n", dev_data[i]->id);
		pthread_create(&read_threads[i], NULL, read_device, dev_data[i]);
		pthread_create(&write_threads[i], NULL, write_device, dev_data[i]);
        i++;
    }

	for (uint16_t i = 0; i < csv_entry_num; i++)
	{
		pthread_barrier_wait(&barrier);
		for (uint8_t j = 0; j < DEVICE_NUM; j++)
		{
			exp_data[j][i].centroid[0] = centroids[j][0];
			exp_data[j][i].centroid[1] = centroids[j][1];
			exp_data[j][i].asc_dir[0] = asc_dirs[j][0];
			exp_data[j][i].asc_dir[1] = asc_dirs[j][1];
		}
		entry_count++;
	}

    for (int j = 0; j < i; j++)
	{
        pthread_join(read_threads[j], NULL);
        pthread_join(write_threads[j], NULL);
		// printf("Thread %u joined\n", dev_data[j]->id);
    }

	for (uint16_t i = 0; i < csv_entry_num; i++)
	{
		printf("[");
		for (uint16_t j = 0; j < DEVICE_NUM; j++)
			printf("[%f,%f],", exp_data[j][i].centroid[0], exp_data[j][i].centroid[1]);
		printf("]\n");
	}

	printf("\n--------------------------------------\n\n");

	for (uint16_t i = 0; i < csv_entry_num; i++)
	{
		printf("[");
		for (uint16_t j = 0; j < DEVICE_NUM; j++)
			printf("[%f,%f],", exp_data[j][i].asc_dir[0], exp_data[j][i].asc_dir[1]);
		printf("]\n");
	}

    for (int j = 0; j < i; j++)
	{
        close(dev_data[j]->fd);
		free(dev_data[j]);
    }

    return 0;
}
