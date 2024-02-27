<a name="readme-top"></a>
<!--
*** Template from https://github.com/othneildrew/Best-README-Template
***
*** LICENSE FROM https://github.com/othneildrew/Best-README-Template:
***
*** MIT License
***
*** Copyright (c) 2021 Othneil Drew
***
*** Permission is hereby granted, free of charge, to any person obtaining a copy
*** of this software and associated documentation files (the "Software"), to deal
*** in the Software without restriction, including without limitation the rights
*** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*** copies of the Software, and to permit persons to whom the Software is
*** furnished to do so, subject to the following conditions:
***
*** The above copyright notice and this permission notice shall be included in all
*** copies or substantial portions of the Software.
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]
[![Project Page][project-shield]][project-url]



<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/UCM-237/Distributed_localization_DWM1001">
    <img src="https://lh5.googleusercontent.com/TQ-S7dGe0BksXoz7WDug6wXFOwrq2ZM4ulpftXcZwXZ7fG6WSJa5c7iHVymqFiFVQmxl3SPCH_l6CMX7YKZKc-0rw0FRhPVFi-n6zSKeEqRPjGPnCooehczFI80BUquHkw=w1280" alt="Logo" width="80" height="80">
  </a>

<h3 align="center">Distributed localization | DWM1001</h3>

  <p align="center">
    Distributed localization system based on the DWM1001 development board.
    <br />
    <a href="https://github.com/UCM-237/Distributed_localization_DWM1001"><strong>Explore the docs »</strong></a>
    <br />
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li><a href="#prerequisites">Prerequisites</a></li>
    <li><a href="#installation">Installation</a></li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

This project aims to develop a distributed localization system in the Decawave DWM-1001 board using UWB communications. It is based in ChibiOS in order to use its RTOS.

[Documentation](https://github.com/UCM-237/Distributed_localization_DWM1001)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



### Built With

* [![ChibiOS][ChibiOS]][ChibiOS-url]
* [![ChibiOS-Contrib][Contrib]][ChibiOS-url]
* [![Jlink][Jlink]][Jlink-url]
* [![Decawave Examples][Decawave]][Decawave-url]


<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
### Prerequisites

Prerequites are mostly inherited from ChibiOS and related to the architecture of the board
* **Jlink**
  
  Download from the official site, [Segger](https://www.segger.com/downloads/jlink/).
  
  For Linux, if the tar package is chosen, the path of the installation should be appended to system **PATH** variable.

* Compiler **arm-none-eabi-gcc**

### Flashing

1. Plug the board through USB and make sure it is detected as a device.
2. Run ```make flash```

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

The main use of this project is to allow swarms of drones to be aware of their position using distributed localization.

_For more examples, please refer to the [Documentation](https://github.com/UCM-237/Distributed_localization_DWM1001)_

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONTRIBUTING -->
## Contributing

Please contact the project maintainers to contribute.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the GNU General Public License v2.0. See `LICENSE` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Swarm Systems Lab - swarmsystemslab@gmail.com

Project Link: [https://github.com/UCM-237/Distributed_localization_DWM1001](https://github.com/UCM-237/Distributed_localization_DWM1001)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/UCM-237/Distributed_localization_DWM1001.svg?style=for-the-badge
[contributors-url]: https://github.com/UCM-237/Distributed_localization_DWM1001/graphs/contributors
[stars-shield]: https://img.shields.io/github/stars/UCM-237/Distributed_localization_DWM1001.svg?style=for-the-badge
[stars-url]: https://github.com/UCM-237/Distributed_localization_DWM1001/stargazers
[issues-shield]: https://img.shields.io/github/issues/UCM-237/Distributed_localization_DWM1001.svg?style=for-the-badge
[issues-url]: https://github.com/UCM-237/Distributed_localization_DWM1001/issues
[license-shield]: https://img.shields.io/github/license/UCM-237/Distributed_localization_DWM1001.svg?style=for-the-badge
[license-url]: https://github.com/UCM-237/Distributed_localization_DWM1001/blob/master/LICENSE.txt
[project-shield]: https://img.shields.io/badge/Swarm_System_Labs-blue
[project-url]: https://sites.google.com/view/hgdemarina
[ChibiOS]: https://img.shields.io/badge/ChibiOS-blue
[ChibiOS-url]: https://github.com/ChibiOS/ChibiOS/tree/stable_21.11.x
[Contrib]: https://img.shields.io/badge/ChibiOS_Contrib-blue
[Contrib-url]: https://github.com/UCM-237/ChibiOS-Contrib/tree/chibios-21.11.x
[Jlink]: https://img.shields.io/badge/Jlink-blue
[Jlink-url]: https://www.segger.com/downloads/jlink/
[Decawave]: https://img.shields.io/badge/DWM1001_Examples-blue
[Decawave-url]: https://github.com/Decawave/dwm1001-examples