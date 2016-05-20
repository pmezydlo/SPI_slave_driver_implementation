<img src="https://travis-ci.org/pmezydlo/SPI_slave_driver_implementation.svg?branch=master" alt=Travis Cl build" />

<h1>SPI slave driver implementation</h1>
This repository contains the code for SPI slave driver realization of my project for GSOC-2016.

<h2>Introduction</h2>
SPI slave driver implementation. The task is to create a driver controlling
SPI hardware controller in slave mode, and to ensure optimal performance through
the use of DMA and interrupt. Creating an easy to implement realization of SPI slave
would definitely help the BeagleBone community members to write applications 
based on SPI much more easily. The first implementation of my protocol driver is going
to example of a bidirectional data exchange. This application will provide 
the BeagleBone community with valuable experience and will be a good example of SPI slave.  
Hardware limitations make it impossible to perform any realization of the device using SPI slave. 
Sending away data to the master during one transaction is not possible. One transaction is enough
to receive data by slave device. To receive and send back data, two transactions are needed. 
The data received from master device in a single transaction is transferred to the user after
completing the transaction. The user's reply to received data is sent in the next transaction.

More information is here:
<a href="https://github.com/pmezydlo/SPI_slave_driver_implementation/wiki"> https://github.com/pmezydlo/SPI_slave_driver_implementation/wiki</a></li>

<h2>License</h2>
SPI slave driver is released under the GPLv2 license.



