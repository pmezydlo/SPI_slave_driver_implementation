<html>
<head>
	<title>SPI slave driver implementation documentation</title>
    <meta charset="UTF-8">
</head>
<body>

    <h1 id="top">SPI slave driver implementation</h1>
	<ol>
	<li><a href="#intro">Introduction</a></li>
	<li><a href="#links">Links</a></li>
	<li><a href="#timeline">Development timeline</a></li>
	<li><a href="#blockdiagram">Block diagram</a></li>	
	<li><a href="#problems">Problems</a></li>
	<li><a href="#buildBBB">Building driver on BeagleBone Board</a></li>	
	<li><a href="#buildx86">Building driver on x86 platform</a></li>
	<li><a href="#book">Books about Linux and LKM</a></li>
	<li><a href="#video">Video presentation</a></li>
	</ol>

	<h2 id="intro">SPI slave driver implementation</h2>
		<p>SPI slave driver implementation. The task is to create a driver controlling SPI hardware controller in slave mode,
		and to ensure optimal performance through the use of DMA and interrupt. Creating an easy to implement realization 
		of SPI slave would definitely help the BeagleBone community members to write applications based on SPI much more 
		easily. The first implementation of my protocol driver is going to example of a bidirectional data exchange. 
		This application will provide the BeagleBone community with valuable experience and will be a good example 
		of SPI slave. Hardware limitations make it impossible to perform any realization of the device using SPI slave. 
		Sending away data to the master during one transaction is not possible. One transaction is enough to receive data 
		by slave device. To receive and send back data, two transactions are needed. The data received from master device 
		in a single transaction is transferred to the user after completing the transaction. The user's reply to received 
		data is sent in the next transaction.</p>
		<ul>
		<li>Student: Patryk Mezydlo</li>
		<li>Mentors: Michael Welling, Andrew Bradford, Matt Porter</li>
		<li>Organization: <a href="https://beagleboard.org/">BeagleBoard.org</a>  </li>
		</ul>
		
	<h2 id="links">Links</h2>
		<ul>	
			<li>Code: <a href="https://github.com/pmezydlo/SPI_slave_driver_implementation">https://github.com/pmezydlo/SPI_slave_driver_implementation</a></li>
			<li>Wiki: <a href="https://github.com/pmezydlo/SPI_slave_driver_implementation/wiki"> https://github.com/pmezydlo/SPI_slave_driver_implementation/wiki</a></li>
			<li>Blog: <a href="http://www.elinux.org/Pmezydlo">http://www.elinux.org/Pmezydlo</a></li>
			<li>GSoC site: <a href=" https://summerofcode.withgoogle.com/projects/#5652864013697024"> https://summerofcode.withgoogle.com/projects/#5652864013697024</a> </li>
			<li>Video presentation: <a href="https://www.youtube.com/watch?v=yBgMwMcvcKg">https://www.youtube.com/watch?v=yBgMwMcvcKg</a></li>
		</ul>	
		
	<h2 id="timeline">Development timeline</h2>	
	<p>Each week I will devote a few hours to write the documentation.<p>
	<ol>
	<li>
		<p>Before the first week (26 May - 28 May)</p>
		<ul ><li>create a GitHub repository</li>
		<li>create eLinux page</li>
		<li>create wiki page</li>
		<li>prepare a few sd card with a Firmware Images</li>
		<li>installation of necessary software</li></ul>
	</li>
	<li>
		<p>Week 1(23 May -29 May)</p>
		<ul><li>skeleton LKM</li>
		<li>create debugging tool</li>
		<li>prepare test code for device SPI(using spidev)</li>
		<li>create DTS for SPI Slave Device settings(for example: which SPI)</li></ul>
	</li>
	<li>
		<p>Week 2(30 May-4 June) </p>
		<ul><li>beginning work on the driver</li>
		<li>create function: probe, remove, init exit</li>
		<li>implementation platform device function</li>
		<li>determinate base address (reading DTS device resources)</li>
		<li>create spi slave structure</li></ul>
	</li>
	<li>
		<p>Week 3(06 June -11 June)</p>
		<ul><li>fill structures</li>
		<li>define McSPI register</li>
		<li>reading and changing registers</li>
		<li>set McSPI registers in slave mode</li></ul>
	</li>
	<li>
		<p>Week 4(13 June-18 June)</p>
		<ul><li>allocate memory for tx and rx buffer</li>
		<li>read and write spi slave buffer(PIO)</li>
		<li>first test SPI slave</li></ul>
	</li>
	<li>
		<p>Week 5(20 June-25 June)</p>
		<ul><li>create interrupt for cs line</li>
		<li>testing cs register (response to changes in the line cs)</li>
		<li>more test with interrupt</li></ul>
	</li>
	<li>
		<p>Week 6(27 June -2 July)</p>
		<ul><li>start working on the dma</li>
		<li>create DMA structure</li>
		<li>create DMA channel</li></ul>
	</li>
	<li>
		<p>Week 7(4 July-9 July)</p>
		<ul><li>configuring dma channel</li>
		<li>interrupt for dma</li>
		<li>create callback functions</li></ul>
	</li>
	<li>
		<p>Week 8(11 July -16 July)</p>
		<ul><li>finally work on DMA</li>
		<li>tests with DMA without API (printk log)</li>
		<li>create framework</li>
		<li>create minor number for charter device</li></ul>
	</li>
	<li>
		<p>Week 9(18 July-23 July)</p>
		<ul><li>create class and device</li> 
		<li>create IOCTL command</li>
		<li>create structure for sending and receiving data using IOCTL</li>
		<li>create mutex for writing and reading</li>
		<li>finally write framework</li>
		<li>create library function</li></ul>
	</li>
	<li>
		<p>Week 10(25 July -5 August)</p>
		<ul><li>documentation, tutorials and examples for SPI Slave framework</li>
		<li>create a make script to compile and install in linux</li> 
		<li>more tests</li></ul>
	</li>
	<li>
		<p>Week 11(6 August -20 August)</p>
		<ul><li>clean up code</li>
		<li>latest tests</li> 
		<li>documentation, tutorials and examples</li></ul>
	</li>


	</ul>
	<h2 id="blockdiagram">Block Diagram</h2>
	<img src="block_diagram.png" alt="Block Diagram" width="1114" height="793"/>
	
	<h2 id="problems">Problems </h2>	
		
	<h2 id="buildBBB">Building on BeagleBone Board</h2>	
		<ol>
			<li>Installing compiler:</li>
			<pre>apt-get install gcc</pre>
			<li>Installing kernel headers:</li>
			<pre>sudo apt-get install linux-headers-'uname -r'</pre>
			<li>Cloning SPI slave repository:</li>
			<pre>git clone git@github.com:pmezydlo/SPI_slave_driver_implementation.git</pre>
			<pre>cd SPI_slave_driver_implementation/</pre>	
			<pre>git checkout</pre>
			<li>Building SPI slave driver:</li>
			<pre>make</pre>				
		</ol>
		
		
		
	<h2 id="buildx86">Building on x86 platform</h2>	
		<ol>
			<li>Installing the arm32 cross-compiler:</li>
			<pre>sudo apt-get update -qq</pre>
			<pre>sudo apt-get install -y gcc-arm-linux-gnueabihf</pre>
			<pre>sudo apt-get install bc </pre>
			<li>Cloning SPI slave repository:</li>
			<pre>git clone git@github.com:pmezydlo/SPI_slave_driver_implementation.git</pre>	
			<pre>cd SPI_slave_driver_implementation/</pre>
			<pre>git checkout</pre>	
			<li>Downloading Linux headers:</li>
			<pre>wget https://github.com/beagleboard/linux/archive/4.4.8-ti-rt-r22.tar.gz</pre>
			<li>Unpacking Linux headers:</li>
			<pre>tar zvxf 4.4.8-ti-rt-r22.tar.gz</pre>
			<li>Installing Linux headers:</li>
			<pre>make -j3 ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- mrproper</pre>
			<pre>make -j3 ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- defconfig</pre>
			<pre>make -j3 ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- modules</pre>	
			<pre>cd ../</pre>
			<li>Building SPI slave driver:</li>
			<pre>make</pre>				
		</ol>
		
	<h2 id="book">Books about Linux and LKM</h2>	
	<ul><li>Linux Kernel Development, Third Edition, Robert Love pdf is here[2]</li>
		<li>LINUX DEVICE DRIVERS, Third Edition, Jonathan Corbet, Alessandro Rubini, and Greg Kroah-Hartman pdf is here[1]</li>
	</ul>
	
	<a href="http://free-electrons.com/doc/books/ldd3.pdf">[1]http://free-electrons.com/doc/books/ldd3.pdf</a></br>
	<a href="https://lwn.net/Kernel/LDD3/">[2]https://lwn.net/Kernel/LDD3/</a>
		
	<h2 id="video">Video presentation</h2>	
		<iframe width="420" height="315"
		src="http://www.youtube.com/embed/yBgMwMcvcKg?autoplay=0">
		</iframe>
	
</body>
</html>

