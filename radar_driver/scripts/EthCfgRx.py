import socket
import struct
import netifaces as ni
import argparse
import ipaddress
import sys


Protocol = 0x11                                     # UDP

# Construct the IPv4 header, encode it into byte strings, and return the encoded header
# Refer to https://en.wikipedia.org/wiki/IPv4 for the structure of IPv4 headers
# Each segment of this IPv4 header is 16 bits long
def GetIPHeader( LocalIP, DestIP ):
    SEG_MASK = 0xFFFF             # This mask is used to trim off values at bit position 16 and greater
    CHECKSUM_POS = 5              # The location of the checksum data within the IPv4 header
    Version = 4                   # IPv4
    HeaderLength = 5
    Dscp = 0x00

    PackSize = 50
    Id = 0x0000
    Flags = 0x00
    FragmentedOffset = 0
    TTL = 64

    HeaderByteData = []
    # Store IPv4 header data into HeaderByteData, where each list element represents 16 bits
    HeaderByteData.append( (((((Version & 0xF) << 4) | (HeaderLength & 0xF)) << 8) | (Dscp & 0xFF)) & SEG_MASK )
    HeaderByteData.append( PackSize & SEG_MASK )
    HeaderByteData.append( Id & SEG_MASK )
    HeaderByteData.append( (((Flags & 0x07) << 13) | (FragmentedOffset & 0x1FFF)) & SEG_MASK )
    HeaderByteData.append( (((TTL & 0xFF) << 8) | (Protocol & 0xFF)) & SEG_MASK )
    HeaderByteData.append( 0 )
    HeaderByteData.append( (((LocalIP[0] & 0xFF) << 8) | (LocalIP[1] & 0xFF)) & SEG_MASK )
    HeaderByteData.append( (((LocalIP[2] & 0xFF) << 8) | (LocalIP[3] & 0xFF)) & SEG_MASK )
    HeaderByteData.append( (((DestIP[0] & 0xFF) << 8) | (DestIP[1] & 0xFF)) & SEG_MASK )
    HeaderByteData.append( (((DestIP[2] & 0xFF) << 8) | (DestIP[3] & 0xFF)) & SEG_MASK )

    # Compute Header Checksum
    Checksum = 0
    for ByteData in HeaderByteData:
        Checksum += ByteData
    Checksum = ((Checksum & 0x000F0000) >> 16) + (Checksum & 0x0000FFFF)
    HeaderByteData[ CHECKSUM_POS ] = (~Checksum) & 0xFFFF

    # Encode IPv4 Header data.
    EncodedHeader = ""
    for ByteData in HeaderByteData:
        EncodedHeader += struct.pack('!1H', ByteData)
    return EncodedHeader



# Construct the UDP payload, encode it into byte strings, and return the encoded payload
def GetPayload( NewTxPort, NewMulticastIP, NewUnicastIP, NewMac ):
    NewRxPort = 31123

    Payload = struct.pack('!4B', NewUnicastIP[0], NewUnicastIP[1], NewUnicastIP[2], NewUnicastIP[3])
    Payload += struct.pack('!6B', NewMac[0], NewMac[1], NewMac[2], NewMac[3], NewMac[4], NewMac[5])
    Payload += struct.pack('!4B', NewMulticastIP[0], NewMulticastIP[1], NewMulticastIP[2], NewMulticastIP[3])
    Payload += struct.pack('!1H', NewTxPort)
    Payload += struct.pack('!4B', 0, 0, 0, 0)         # Reserved Bytes
    Payload += struct.pack('!1H', NewRxPort)

    return Payload

# Input Arguments -- IPString: String representing an IP Address
# Return          -- Size-4 Array of unsigned integers from the IP Address
def ConvertIP(IPString):
    ConvertedIP = IPString.split('.')
    for i in range(len(ConvertedIP)):
        ConvertedIP[i] = int(ConvertedIP[i])
    return ConvertedIP

# Input Arguments -- MACString: String representing a MAC Address
# Return          -- Size-6 Array of unsigned integers from the MAC Address
def ConvertMac(MacString):
    if MacString.count('.') == 5:
        MacArray = MacString.split('.')
    elif MacString.count(':') == 5:
        MacArray = MacString.split(':')
    else:
        sys.stderr.write("Error: You entered an invalid MAC Address. Required format: x:x:x:x:x:x  or  x.x.x.x.x.x  {0 <= x <= FF}\n")
        sys.exit(1)

    for i in range(len(MacArray)):
        MacArray[i] = int(MacArray[i], 16)
        if not ((MacArray[i] <= 0xFF) and (MacArray[i] >= 0)):
            sys.stderr.write("Error: You entered an invalid MAC Address. Required format: x:x:x:x:x:x  or  x.x.x.x.x.x  {0 <= x <= FF}\n")
            sys.exit(1)
    return MacArray

def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--ethname', required=True, metavar='"Ethernet Interface Name"', type=str, help='Type "ifconfig" into the terminal for a list of interfaces to use. Example: enp3s0')
    parser.add_argument('-ip', '--currentip', required=True, metavar='"Current Radar IP"', type=str, help="Input Format: x.x.x.x  [0 <= x <= 255]. Example: 192.168.0.1")
    parser.add_argument('-m', '--currentmac', required=True, metavar='"Current Radar MAC"', type=str, help="Input Format: x:x:x:x:x:x  or  x.x.x.x.x.x  [0 <= x <= FF, x is a 2-digit hex number]. Example: 02:08:02:03:FF:00")
    parser.add_argument('-tx', '--txport', required=False, metavar='"New Radar Tx Port"', type=int, help='Allowed Range: 31115 - 31122, inclusive', default=31122)
    parser.add_argument('-ui', '--unicastip', required=False, metavar='"New Radar IP Address -- Unicast"', type=str, help='Input Format:  x.x.x.x  [0 <= x <= 255]. Example: 192.168.0.1', default=None)
    parser.add_argument('-mi', '--multicastip', required=False, metavar='"New Radar IP Address -- Multicast"', type=str, help='Input Format:  x.x.x.x  [0 <= x <= 255]. Example: 225.0.0.1', default='225.0.0.1')
    parser.add_argument('-nm', '--newmac', required=False, metavar='"New Radar MAC Address"', type=str, help='Input Format: x:x:x:x:x:x  or  x.x.x.x.x.x  [0 <= x <= FF, x is a 2-digit hex number]. Example: 02:08:02:03:FF:00', default=None)
    args = parser.parse_args()

    try:
        ipaddress.ip_address(unicode(args.currentip))
        CurrentRadarIP = ConvertIP(args.currentip)                  # Return an array (size 4) of unsigned integers representing the IP Address

        ipaddress.ip_address(unicode(args.multicastip))
        NewRadarMulticastIP = ConvertIP(args.multicastip)

        if (args.unicastip == None):
            args.unicastip = args.currentip
            NewRadarUnicastIP = ConvertIP(args.currentip)           # Set the new Radar IP to the current Radar IP if user specified nothing
        else:
            ipaddress.ip_address(unicode(args.unicastip))
            NewRadarUnicastIP = ConvertIP(args.unicastip)

    except ValueError:
        sys.stderr.write("Error: You entered an invalid IP. Required format:  x.x.x.x  {0 <= x <= 255}\n")
        sys.exit(1)

    CurrentRadarMAC = ConvertMac(args.currentmac)                   # Return an array (size 6) of unsigned integers representing the MAC Address
    if (args.newmac == None):
        args.newmac = args.currentmac
        NewRadarMac = ConvertMac(args.currentmac)                   # Set the new Radar MAC to the current MAC if user specified nothing
    else:
        NewRadarMac = ConvertMac(args.newmac)

    if ((args.txport < 31115) or (args.txport > 31122)):
        sys.stderr.write("Error: You entered an invalid Tx Port. Number must be from 31115 to 31122, inclusive.\n")
        sys.exit(1)

    # Initialize Connection Properties
    InterfaceName = args.ethname
    NetInfo = ni.ifaddresses(InterfaceName)
    LocalMacString = NetInfo[ni.AF_LINK][0]['addr']
    # Convert LocalMacString to an array of integers
    LocalMAC = [ int(LocalMacString[0:2], 16), int(LocalMacString[3:5], 16), int(LocalMacString[6:8], 16), int(LocalMacString[9:11], 16), int(LocalMacString[12:14], 16), int(LocalMacString[15:17], 16) ]

    LocalIPString = NetInfo[ni.AF_INET][0]['addr']
    LocalIP = []                                        # Convert LocalIPString to an array of integers
    for i in range(3):
        pos = LocalIPString.find('.')
        LocalIP.append( int(LocalIPString[:pos]) )
        LocalIPString = LocalIPString[pos+1:]
    LocalIP.append( int(LocalIPString) )

    CurrentRadarRxPort = 31123                          # Current Destination Rx Port
    LocalTxPort = 31123                                 # Current local machine Tx Port
    EtherType = [0x08, 0x00]                            # Ethernet Communication Type (IPv4)
    Length = 0x001e                                     # Length of the UDP Payload (hard-coded to be 0x001e for now)


    # Build the Ethernet Frame
    EthFrame = struct.pack('!6B', CurrentRadarMAC[0], CurrentRadarMAC[1], CurrentRadarMAC[2], CurrentRadarMAC[3], CurrentRadarMAC[4], CurrentRadarMAC[5])
    EthFrame += struct.pack('!6B', LocalMAC[0], LocalMAC[1], LocalMAC[2], LocalMAC[3], LocalMAC[4], LocalMAC[5])
    EthFrame += struct.pack('!2B', EtherType[0], EtherType[1])
    EthFrame += GetIPHeader( LocalIP, CurrentRadarIP)
    EthFrame += struct.pack('!1H', LocalTxPort)
    EthFrame += struct.pack('!1H', CurrentRadarRxPort)
    Payload = GetPayload( args.txport, NewRadarMulticastIP, NewRadarUnicastIP, NewRadarMac )


    # Compute UDP checksum
    CRC = ((LocalIP[0] << 8) | LocalIP[1]) + ((LocalIP[2] << 8) | LocalIP[3])
    CRC += ((CurrentRadarIP[0] << 8) | CurrentRadarIP[1]) + ((CurrentRadarIP[2] << 8) | CurrentRadarIP[3])
    CRC += Protocol + Length
    for i in range(len(Payload)/2):                     # Add Data as 16-bit values
        val = Payload[(2*i) : (2*i) + 2]
        CRC += struct.unpack('!1H', val)[0]
    CRC += Length + LocalTxPort + CurrentRadarRxPort

    while (CRC > 0x0000FFFF):                           # Reduce CRC to a 16-bit value
        CRC =  (CRC & 0x0000FFFF) + ((CRC & 0xFFFF0000) >> 16)
    CRC = ~CRC & 0xFFFF
    #print("UDP Checksum : "+format(CRC, '04x'))


    # Create the final packet, which includes the IPv4 header, payload, and checksums
    Packet = EthFrame + struct.pack('!1H', Length) + struct.pack('!1H', CRC) + Payload
    # Send packet to the RADAR
    sock = socket.socket(socket.AF_PACKET,
                        socket.SOCK_RAW, 0x0800)
    sock.bind((InterfaceName, 0x0800))
    print("\nFinished sending a packet of "+str(sock.send(Packet))+" Bytes.")
    print("New Radar MAC: "+args.newmac)
    print("New Radar Unicast IP: "+args.unicastip)
    print("New Radar Multicast IP: "+args.multicastip)
    print("New Radar Tx Port: "+str(args.txport)+"\n")

if __name__ == "__main__":
    main()
