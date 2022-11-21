/*
 * udp.cpp
 *
 *  Created on: 9 May, 2017
 *      Author: sylam
 */

#include <iostream>
#include <sstream>
#include <iomanip>
#include <netinet/in.h>	/* needed for sockaddr_in */
#include <arpa/inet.h>	/* defines inet_aton */

#define DESTINATION_IP "192.168.0.255"
#define PORT 3105

using namespace std;

static struct sockaddr_in remaddr;
static int fd;

//http://stackoverflow.com/questions/2212776/overload-handling-of-stdendl
class UDPstream: public ostream {
public:
	UDPstream(void): ostream(&buffer) {
		setf(hex, basefield); fill('0');
	    /* create a socket */
		if ((fd=socket(AF_INET, SOCK_DGRAM, 0))==-1)
			cerr << "cannot create socket" << endl;
	    /*  enable boardcast */
	    int broadcastEnable=1;
	    setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
	    /* now define remaddr, the address to whom we want to send messages */
		/* For convenience, the host address is expressed as a numeric IP address */
		/* that we will convert to a binary format via inet_aton */
		remaddr.sin_family = AF_INET;
		remaddr.sin_port = htons(PORT);
		if (inet_aton(DESTINATION_IP, &remaddr.sin_addr)==0) {
			cerr << "inet_aton() failed" << endl;
			fd = -1;
		}
		if (fd == -1) exit(1);
	}
private:
    class Buffer: public stringbuf {
        virtual int sync(void);
    } buffer;
} udp;

int UDPstream::Buffer::sync(void) {
	unsigned char cs = 0;
	string s = str();
	for (unsigned int i = 0; i < s.length()-1; i++) cs ^= s[i]; //except '\n'
	pubseekoff(-1, ios_base::end);	// step back to remove '\n'
	udp << setw(3) << (int)cs;
//	udp << "\n";			// DON'T use endl that will invoke sync again
	s = str(); str("");
	if (fd != -1) if (sendto(fd, s.c_str(), s.length(), 0, (struct sockaddr *)&remaddr, sizeof(remaddr))==-1)
    	cerr << "sendto" << endl;
    return 0;
}

// socat STDIO UDP4-DATAGRAM:192.168.0.255:1500,broadcast
// socat -u udp-recv:3105 -


