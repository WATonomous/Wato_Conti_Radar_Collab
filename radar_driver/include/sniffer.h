#ifndef SNIFFER_H
#define SNIFFER_H

int run(int port, int packets, char interface[], const char filter[], int capture_live, const char *capture_path);

#endif /* SNIFFER_H */
