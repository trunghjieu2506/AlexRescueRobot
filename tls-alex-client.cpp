
// Routines to create a TLS client
#include "make_tls_client.h"

// Network packet types
#include "netconstants.h"

// Packet types, error codes, etc.
#include "constants.h"

// buffer length is 128 bytes
#define BUF_LEN 128

// Tells us that the network is running.
static volatile int networkActive=0;

void handleError(const char *buffer)
{
	switch(buffer[1])
	{
		case RESP_OK:
			printf("Command / Status OK\n");
			break;

		case RESP_BAD_PACKET:
			printf("BAD MAGIC NUMBER FROM ARDUINO\n");
			break;

		case RESP_BAD_CHECKSUM:
			printf("BAD CHECKSUM FROM ARDUINO\n");
			break;

		case RESP_BAD_COMMAND:
			printf("PI SENT BAD COMMAND TO ARDUINO\n");
			break;

		case RESP_BAD_RESPONSE:
			printf("PI GOT BAD RESPONSE FROM ARDUINO\n");
			break;

		default:
			printf("PI IS CONFUSED!\n");
	}
}

void handleStatus(const char *buffer)
{
	int32_t data[16];
	memcpy(data, &buffer[1], sizeof(data));

	printf("\n ------- ALEX STATUS REPORT ------- \n\n");
	printf("Colur Detected:\t\t%d\n", data[0]);
	printf("R:\t\t%d\n", data[1]);
	printf("G:\t\t%d\n", data[2]);
	printf("B:\t\t%d\n", data[3]);
	printf("Left Forward Ticks Turns:\t%d\n", data[4]);
	printf("Right Forward Ticks Turns:\t%d\n", data[5]);
	printf("Left Reverse Ticks Turns:\t%d\n", data[6]);
	printf("Right Reverse Ticks Turns:\t%d\n", data[7]);
	printf("Forward Distance:\t\t%d\n", data[8]);
	printf("Reverse Distance:\t\t%d\n", data[9]);
	// printf("Colour detected: \t\t%d\n", data[10]);
	printf("\n---------------------------------------\n\n");
}

void handleMessage(const char *buffer)
{
	printf("MESSAGE FROM ALEX: %s\n", &buffer[1]);
}

void handleCommand(const char *buffer)
{
	// We don't do anything because we issue commands
	// but we don't get them. Put this here
	// for future expansion
}

void handleNetwork(const char *buffer, int len)
{
	// The first byte is the packet type
	int type = buffer[0];

	switch(type)
	{
		case NET_ERROR_PACKET:
		handleError(buffer);
		break;

		case NET_STATUS_PACKET:
		handleStatus(buffer);
		break;

		case NET_MESSAGE_PACKET:
		handleMessage(buffer);
		break;

		case NET_COMMAND_PACKET:
		handleCommand(buffer);
		break;
	}
}

void sendData(void *conn, const char *buffer, int len)
{
	int c;
	printf("\nSENDING %d BYTES DATA\n\n", len);
	if(networkActive)
	{
		c = sslWrite(conn, buffer, len);
		networkActive = (c > 0);
	}
}

void *readerThread(void *conn)
{
	char buffer[128];
	int len;

	while(networkActive)
	{

		len = sslRead(conn,buffer,sizeof(buffer));
        printf("read %d bytes from server.\n", len);

		networkActive = (len > 0);

		if(networkActive)
			handleNetwork(buffer, len);
	}

	printf("Exiting network listener thread\n");

	EXIT_THREAD(conn);

    return NULL;
}

void flushInput()
{
	char c;

	while((c = getchar()) != '\n' && c != EOF);
}

void getParams(int32_t *params)
{
	printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
	printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
	scanf("%d", &params[0]);
	flushInput();
}

void *writerThread(void *conn)
{
	int quit=0;

	while(!quit)
	{
		char ch;
		printf("Command (w=forward, s=reverse, a=turn left, d=turn right, j=stop, c=clear stats, g=get colour q=exit)\n");
		scanf("%c", &ch);

		// Purge extraneous characters from input stream
		flushInput();

		char buffer[10];
		int32_t params[2];

		buffer[0] = NET_COMMAND_PACKET;
		switch(ch)
		{
			case 'w':
			case 'W':
			case 's':
			case 'S':
						params[0]= 10;
						params[1]= 70; //50
						buffer[1] = ch;
						memcpy(&buffer[2], params, sizeof(params));
						sendData(conn, buffer, sizeof(buffer));
						break;
			case 'a':
			case 'A':
						params[0]= 90;
						params[1]= 70; //40
						buffer[1] = ch;
						memcpy(&buffer[2], params, sizeof(params));
						sendData(conn, buffer, sizeof(buffer));
						break;
			case 'd':
			case 'D':
						params[0]= 90;
						params[1]= 70; //50
						buffer[1] = ch;
						memcpy(&buffer[2], params, sizeof(params));
						sendData(conn, buffer, sizeof(buffer));
						break;
			case 'j':
			case 'J':
			case 'l':
			case 'L':
						getParams(params);
						params[1] = 70; //50
						buffer[1] = ch;
						memcpy(&buffer[2], params, sizeof(params));
						sendData(conn, buffer, sizeof(buffer));
						break;
			case 'h':
			case 'H':
			case 'c':
			case 'C':
			case 'g':
			case 'G':
					params[0]=0;
					params[1]=0;
					memcpy(&buffer[2], params, sizeof(params));
					buffer[1] = ch;
					sendData(conn, buffer, sizeof(buffer));
					break;
			case 'q':
			case 'Q':
				quit=1;
				break;
			default:
				printf("BAD COMMAND\n");
		}
	}

	printf("Exiting keyboard thread\n");

	EXIT_THREAD(conn);

    return NULL;
}
   
#define SERVER_NAME "172.20.10.10"
#define CA_CERT_FNAME "signing.pem"
#define PORT_NUM 5001
#define CLIENT_CERT_FNAME "laptop.crt"
#define CLIENT_KEY_FNAME "laptop.key"
#define SERVER_NAME_ON_CERT "amir"

void connectToServer(const char *serverName, int portNum)
{
    createClient(SERVER_NAME, PORT_NUM, 1, CA_CERT_FNAME, SERVER_NAME_ON_CERT, 1, CLIENT_CERT_FNAME, CLIENT_KEY_FNAME, readerThread, writerThread);
}

int main(int ac, char **av)
{
	if(ac != 3)
	{
		fprintf(stderr, "\n\n%s <IP address> <Port Number>\n\n", av[0]);
		exit(-1);
	}

    networkActive = 1;
    connectToServer(av[1], atoi(av[2]));

	while(client_is_running());
	
	printf("\nMAIN exiting\n\n");
}
