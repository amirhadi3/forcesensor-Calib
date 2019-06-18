#n-bit CRC checksum generation and checking based on algorithm in Wikipedia
#

def crc(p, n=32, polynomial=0xedb88320):
	"""
    Carry out XOR + shifting routine for n-bit CRC Checksum creation and checking
    :param n the number of bits in the checksum (default is 32)
    :param p the bit sequence in hex or int which includes the number whose checksum it is
    	and either n zeros or the crc to check appended to the end
    :param polynomial the bit string of the CRC polynomial to use
    	Default is the typical CRC-32 polynomial. CRC-4 often uses 0x0C
    :return p after all the shifting and XOR-ing
    	If checking a CRC, p should be 0
    	If creating a CRC, p's last n bytes are the CRC-n checksum
    """
    
    #Store the binary representation of p and the number of bits
    pBin = bin(p)[2:]
    length = len(pBin)

    #When p gets smaller than this, the dividend is zero (see Wikipedia)
    minVal = 2**n 

    #Shift the polynomial to align with the most significant bit
    poly = polynomial << (length - len(bin(polynomial)) + 2) #Plus 2 is for the extra 2 characters in the binary string: 0b...
    i = 0
    
    #Terminate when the dividend is equal to zero and only the checksum portion remains
    while p >= minVal: 
    	#Shift the divisor until it is aligned with the most significant 1 in p (in binary)
        while pBin[i] == '0' and i <= length - n: 
            i = i + 1
            poly = poly >> 1

        #XOR the number with the divisor
        p = p ^ poly 
        #Update the bit string (used in the loop above)
        pBin = bin(p)[2:]
        #Make sure leading zeros weren't removed by Python
        while len(pBin) < length:
            pBin = '0' + pBin

    return p

def calc_crc(p, n=32, polynomial=0xedb88320):
	"""
    Create n-bit CRC Checksum
    :param n the number of bits in the checksum (default is 32)
    :param p the bit sequence in bytes, hex, or int to create the checksum for 
    	For example, bytes 0 to 46 in the 51 byte sensor packet
    :param polynomial the bit string of the CRC polynomial to use
    	Default is the typical CRC-32 polynomial. CRC-4 often uses 0x0C
    :return The n-bit checksum
    """

    #Convert to correct type and append an n bit buffer of 0s
    if type(p) == bytes:
        p = int(p.hex(),16) << n
    else:
        p = p << n
    
   pBin = bin(crc(p, n, polynomial))

	#Return the n-bit CRC checksum (last n bits of p)
    return int(pBin[n:],2) 

def check_crc(crc, p, n=32, polynomial=0xedb88320):
	    """
	    Check CRC Checksum with arbitrary number of bits
	    :param crc the n bit checksum in hex, int, or bytes
	    :param p the bit sequence to compare to the checksum. (For example, bytes 0 to 46 in the 51 byte sensor packet)
	    :param polynomial the bit string of the CRC polynomial to use
	    	Default is the typical CRC-32 polynomial. CRC-4 often uses 0x0C
	    :return True if the checksum matches, False otherwise
	    """
	    
	    #Convert everything to correct type
	    if type(p) == bytes:
	   		p = int(p.hex(),16)

	   	#Append the crc to the end of p (least significant side)
	    p = (p << n) + crc

	    p = crc(p, n, polynomial)
	    
	    #After all the shifting and XOR-ing is complete, p should equal 0
	    return p == 0