import SPIslave

spi = SPIslave.SPIslave()
spi.open(0)

spi.bits_per_word = 8
