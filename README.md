# rx888_stream
Outputs samples to stdout. To build run `make`

Example command: `./rx888_stream -f SDDC_FX3.img -g 3 --samplerate 135000000 > test.dat`

Rewrite of: [rx888_test](https://github.com/cozycactus/rx888_test)

For limiting the sample filesize, try adding the the pv command to your pipe command line:
    <br>`./rx888_stream -f SDDC_FX3.img10MHz -s 135000000 -g 3 --refclock-10M | pv -Ss 20M > test_135m0_10MHzOCXO`<br>
the command: pv -Ss <size> will kill the stream at a fixed size.

For rapidly selecting the 10 MHz vs 27 MHz refclocks, there is a new option in rx888_stream:

 --refclock-10M or -T 
 
 will configure for 10 MHz refclock
 
 However, you still have to supply the correct image file (ending 10MHz)
