/* General Use Includes */
#include "processPacket.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <getopt.h>
#include <signal.h>
#include <stdint.h>
#include <fstream>

/* Libpcap includes */
#include <pcap.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <netinet/ip_icmp.h>

//Reference Blog:
//https://vichargrave.github.io/programming/develop-a-packet-sniffer-with-libpcap/

pcap_t* pd; //pcap file descriptor
int linkhdrlen; //Datalink hdr len (so we can skip it)

pcap_t* open_pcap_socket(char* device, const char* bpfstr)
{
    char errbuf[PCAP_ERRBUF_SIZE]; //General buffer for error messages
    pcap_t* pd;
    uint32_t  srcip, netmask;
    struct bpf_program  bpf;

    // If no network interface (device) is specfied, get the first one.
    if (!*device && !(device = pcap_lookupdev(errbuf)))
    {
        printf("pcap_lookupdev(): %s\n", errbuf);
        return NULL;
    }

    printf("Using network Interface: %s\n", device);

    // Open the device for live capture, as opposed to reading a packet
    // capture file.
    if ((pd = pcap_open_live(device, BUFSIZ, 1, 0, errbuf)) == NULL) //BUFSIZ = 8192 bytes
    {
        printf("Try running the following if permission errors:\n");
        printf("\tsudo setcap 'cap_net_raw=pe' devel/lib/radar_driver/radar_publisher\n");
        printf("pcap_open_live(): %s\n", errbuf);
        return NULL;
    }

    // Get network device source IP address and netmask.
    if (pcap_lookupnet(device, &srcip, &netmask, errbuf) < 0)
    {
        printf("pcap_lookupnet: %s\n", errbuf);
        return NULL;
    }

    printf("Connecting to IP: %d and Mask: %d\n", srcip, netmask);

    // Convert the packet filter epxression into a packet
    // filter binary. Setup packet filter expression
    if (pcap_compile(pd, &bpf, (char*)bpfstr, 0, netmask))
    {
        printf("pcap_compile(): %s\n", pcap_geterr(pd));
        return NULL;
    }

    // Assign the packet filter to the given libpcap socket.
    if (pcap_setfilter(pd, &bpf) < 0)
    {
        printf("pcap_setfilter(): %s\n", pcap_geterr(pd));
        return NULL;
    }

    return pd;
}

pcap_t* open_pcap_file(const char* filename)
{
    char errbuf[PCAP_ERRBUF_SIZE]; //General buffer for error messages
    pcap_t* pd;
    uint32_t  srcip, netmask;
    struct bpf_program  bpf;

    // Open the device for pcap readout
    if ((pd = pcap_open_offline_with_tstamp_precision(filename, PCAP_TSTAMP_PRECISION_MICRO, errbuf)) == NULL) //BUFSIZ = 8192 bytes
    {
        printf("Failed to open pcap file: %s\r\n", filename);
        printf("pcap_open_offline(): %s\r\n", errbuf);
        return NULL;
    }

    return pd;
}

void capture_loop(pcap_t* pd, int packets, pcap_handler func, int capture_live, const char *capture_path)
{
    int linktype;
 
    // Determine the datalink layer type.
    if ((linktype = pcap_datalink(pd)) < 0)
    {
        printf("pcap_datalink(): %s\n", pcap_geterr(pd));
        return;
    }
 
    // Set the datalink layer header size.
    switch (linktype)
    {
    case DLT_NULL:
        linkhdrlen = 4;
        break;
 
    case DLT_EN10MB:
        linkhdrlen = 14;
        break;
 
    case DLT_SLIP:
    case DLT_PPP:
        linkhdrlen = 24;
        break;
 
    default:
        printf("Unsupported datalink (%d)\n", linktype);
        return;
    }
    
    // Start capturing packets.
    if (capture_live == LIVE) {
        if (pcap_loop(pd, packets, func, 0) < 0) {
            printf("pcap_loop failed: %s\n", pcap_geterr(pd));
        }
    } else {
        while(1) {
            if (pcap_dispatch(pd, packets, func, 0) < 0) { //Read packets till end of file
                printf("pcap_loop failed: %s\n", pcap_geterr(pd));
                break;
            }

            if (!(pd = open_pcap_file(capture_path))) { //Reset state of pcap_t struct
                printf("Struct Reset from %s failed\n", capture_path);
                break;
            }
        }
    }
}

void receive_packet(u_char *user, struct pcap_pkthdr *packethdr, 
                  u_char *packetptr)
{
    struct ip* iphdr;
    struct udphdr* udphdr;
    char iphdrInfo[256], srcip[256], dstip[256];
    unsigned short id, seq;
 
    // Skip the datalink layer header and get the IP header fields.
    packetptr += linkhdrlen; //Forward packet pointer past datalink layer header
    iphdr = (struct ip*)packetptr; //Load into iphdr struct
    strcpy(srcip, inet_ntoa(iphdr->ip_src));
    strcpy(dstip, inet_ntoa(iphdr->ip_dst));
    sprintf(iphdrInfo, "ID:%d TOS:0x%x, TTL:%d IpLen:%d DgLen:%d",
            ntohs(iphdr->ip_id), iphdr->ip_tos, iphdr->ip_ttl,
            4*iphdr->ip_hl, ntohs(iphdr->ip_len));
 
    // Advance to the transport layer header then parse and display
    // the fields based on the type of hearder: tcp, udp or icmp.
    packetptr += 4*iphdr->ip_hl;
    if (iphdr->ip_p != IPPROTO_UDP) {
        return; //Actually skip it
    } else {
        udphdr = (struct udphdr*)packetptr; //Load into UDP hdr struct
        parse_packet(udphdr, packetptr); //Pass of to our parser
    }
}

void bailout(int signo)
{
    struct pcap_stat stats;
 
    if (pcap_stats(pd, &stats) >= 0)
    {
        printf("%d packets received\n", stats.ps_recv);
        printf("%d packets dropped\n\n", stats.ps_drop);
    }
    pcap_close(pd);
    exit(0);
}

int run(int port, int packets, char interface[], const char filter[], int capture_live, const char *capture_path) {
    printf("Starting Init Now\n");
    char bpfstr[256] = ""; //Berkeley Packet Filter String

    //Hardcode to use UDP. Enter the protocol, port and whatever filter string we're given
    snprintf(bpfstr, sizeof(bpfstr), "udp port %d %s", port, filter);
    printf("You entered: %s: %s\n", interface, bpfstr); //Spit back out to the user

    // Open libpcap, set the program termination signals then start
    // processing packets for LIVE data.
    if ((capture_live == LIVE) && (pd = open_pcap_socket(interface, bpfstr))) {
        signal(SIGINT, bailout); //Set function callbacks for various failure signals
        signal(SIGTERM, bailout);
        signal(SIGQUIT, bailout);

        //Now we call our looping function to capture packets
        //We give it the file descriptor for the socket, the # of packets to collect
        //And a call back for us to parse the packets coming in
        capture_loop(pd, packets, (pcap_handler)receive_packet, capture_live, capture_path);
        bailout(0); //Cut out
    } else if ((capture_live == OFFLINE) && (pd = open_pcap_file(capture_path))) {
        signal(SIGINT, bailout); //Set function callbacks for various failure signals
        signal(SIGTERM, bailout);
        signal(SIGQUIT, bailout);

        //Now we call our looping function to capture packets
        //We give it the file descriptor for the socket, the # of packets to collect
        //And a call back for us to parse the packets coming in
        capture_loop(pd, 0, (pcap_handler)receive_packet, capture_live, capture_path);
        bailout(0); //Cut out
    }

    return -1;
}
