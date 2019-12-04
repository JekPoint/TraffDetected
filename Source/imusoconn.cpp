#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <inttypes.h>
#include <imusoconn.h>

#define CRC_CHECK               1

#define PROCESS_OUT_BINARY      0
#define PROCESS_OUT_HUMAN       1

#define STATE_WAIT              0
#define STATE_START_RECEIVED    1

#define CMD_GET_VERSION            0x00
#define CMD_SET_DAY_MODE           0x01
#define CMD_SET_NIGHT_MODE         0x02
#define CMD_SET_CURRENT_SW_VALUE   0x03
#define CMD_GET_SWITCH_VALUE       0x04
#define CMD_SET_HYSTER_VALUE       0x05
#define CMD_GET_HYSTER_VALUE       0x06
#define CMD_SET_CAMERA_SWITCH      0x07
#define CMD_GET_CAMERA_SWITCH      0x08
#define CMD_SET_CAM_PROTOCOL       0x09
#define CMD_GET_CAM_PROTOCOL       0x0A
#define CMD_SET_OUTER_SW_VALUE     0x0B
#define CMD_GET_CUR_ADC            0x0C
#define CMD_EEPROM_INIT            0x0D
#define CMD_SET_CUR_MODE           0x0E
#define CMD_GET_LIGTH_DURATION     0x0F
#define CMD_SET_LIGTH_DURATION     0x10
#define CMD_MAX             	   0x10

int last_command;
int buffer_processed;
int process_kind = PROCESS_OUT_BINARY;
struct imuso_command
{
	char * text_cmd;
	char * text_comment;
	uint8_t cmd;

};

struct imuso_command imuso_commands[21];

struct imuso_command create_command(int cmd, char* text_cmd, char * text_comment)
{
	struct imuso_command _cmd;
	_cmd.cmd = cmd;
	_cmd.text_cmd = text_cmd;
	_cmd.text_comment = text_comment;
	return _cmd;
}

int set_interface_attribs(int fd, int speed, int parity)
{
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0)
	{
		// error_message ("error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);


	tty.c_cflag = ((tty.c_cflag & ~CSIZE) | CS8);     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	cfmakeraw(&tty);
	tty.c_iflag &= ~IGNBRK;         // ignore break signal
	tty.c_lflag = 0;                // no signaling chars, no echo,
	// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN] = 0;            // read doesn't block
	tty.c_cc[VTIME] = 2;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
	// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;



	if (tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		//error_message ("error %d from tcsetattr", errno);
		return -1;
	}
	return 0;
}

void set_blocking(int fd, int should_block)
{
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0)
	{
		// error_message ("error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN] = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 2;            // 0.5 seconds read timeout

	// if (tcsetattr (fd, TCSANOW, &tty) != 0)
	//  error_message ("error %d setting term attributes", errno);
}


int state;
uint8_t receivebuf[8];
int bufpos;
uint8_t datalength;
uint8_t crc_byte(uint8_t crc, uint8_t data)
{
	uint8_t i;
	crc = crc^data;
	for (i = 0; i < 8; i++)
	{
		if (crc & 0x01)
			crc = (crc >> 1) ^ 0x8C;
		else
			crc = crc >> 1;
	}
	return crc;
}

int set_state(int newstate)
{
	state = newstate;
	if (newstate == STATE_WAIT) bufpos = 0;
	return 0;
}




int processbuffer(uint8_t * buf, int length, int process_kind)
{
	if (process_kind == PROCESS_OUT_BINARY){
		int i;
		for (i = 0; i<length; i++) printf("%.2X ", buf[i]);
		printf("\n");
	}
	if (process_kind == PROCESS_OUT_HUMAN){
		switch (buf[2])
		{
			int i;
		case CMD_GET_VERSION:
			printf("Firmware version: %d.%d\n", buf[4], buf[3]);
			break;
		case CMD_GET_SWITCH_VALUE:
			printf("%d\n", (buf[4] << 8) + buf[3]);
			break;
		case CMD_GET_CUR_ADC:
			printf("%d\n", (buf[4] << 8) + buf[3]);
			break;
		case CMD_GET_HYSTER_VALUE:
			printf("%d\n", (buf[4] << 8) + buf[3]);
			break;
		case CMD_GET_CAM_PROTOCOL:

			switch (buf[3]){
			case 0: printf("PELCO-D\n");
				break;
			case 1: printf("PELCO-P\n");
				break;
			case 2: printf("SAMSUNG-E\n");
				break;
			case 3: printf("SAMSUNG-T\n");
				break;
			}

			break;

		case CMD_GET_CAMERA_SWITCH:

			for (i = 0; i<8; i++)
			{
				printf("%d ", (buf[3] >> i) & 1);
			}
			for (i = 0; i<8; i++)
			{
				printf("%d ", (buf[4] >> i) & 1);
			}
			printf("\n");
			break;

		default:
		{
				   processbuffer(buf, length, PROCESS_OUT_BINARY);
		}


		}

	}
	return 0;
}

int processbyte(uint8_t b)
{
	if (state == STATE_WAIT){
		if (b == 0xFF) {
			set_state(STATE_START_RECEIVED);

		}

	}
	//printf("received %X %d\n",b, bufpos);
	if (state == STATE_START_RECEIVED){
		receivebuf[bufpos] = b;
		//printf("received %X %d\n",b, bufpos);

		if (bufpos >= 6){



			//  printf("bufpos = %d, datalength = %d\n",bufpos,datalength);
			int crc = 0;
			uint8_t i;
			for (i = 1; i<bufpos; i++)
			{
				crc = crc + receivebuf[i];
			}
			crc = crc & 0xFF;
			/*for (i = 0; i<=bufpos; i++)
			{
			printf(" %.2X",receivebuf[i]);
			}*/

			if ((b == crc) || (!CRC_CHECK)){

				if (receivebuf[1] == 0xFE)
				{
					processbuffer(receivebuf, bufpos + 1, process_kind);
					buffer_processed = 1;
				};

			}
			else printf("CRC error\n");

			set_state(STATE_WAIT);
		}
		else bufpos++;

	}
	return 0;
}


void prepare_pelco_d_command(uint8_t *inbuf, uint8_t *outbuf)
{
	outbuf[0] = 0xFF;
	outbuf[1] = 0x20;
	memcpy(outbuf + 2, inbuf, 4);

	int i;
	int crc = 0;
	for (i = 1; i<6; i++)
	{
		crc = crc + outbuf[i];
		//printf("%.2X ",outbuf[i]);
	}
	//printf("\n");
	outbuf[6] = crc & 0xFF;
}

void print_usage()
{
	printf("usage: imusoconn device command [--human|--help] parameters \n");
	printf("command: \n");

	int i;
	for (i = 0; i <= CMD_MAX; i++)
	{
		printf("%s\n", imuso_commands[i].text_cmd);
	}
	printf("for help print: imusoconn device command --help\n");
}



void init_commands_list()
{
	imuso_commands[0x00] = create_command(0x00, "--get-version", "return version of IMUSO furmware\n");
	imuso_commands[0x01] = create_command(0x01, "--set-day-mode", "set IMUSO to day mode for 1 minute\n");
	imuso_commands[0x02] = create_command(0x02, "--set-night-mode", "set IMUSO to night mode for 1 minute\n");
	imuso_commands[0x03] = create_command(0x03, "--set-current-sw-value", "set current light value to switch value\n");
	imuso_commands[0x04] = create_command(0x04, "--get-switch-value", "get d/n switch value\n");
	imuso_commands[0x05] = create_command(0x05, "--set-hyster-value", "set switch hysteresis value\n");
	imuso_commands[0x06] = create_command(0x06, "--get-hyster-value", "get switch hysteresis value\n");
	imuso_commands[0x07] = create_command(0x07, "--set-camera-mask", "set mask of cameras adresses\n");
	imuso_commands[0x08] = create_command(0x08, "--get-camera-mask", "get mask of cameras adresses\n");
	imuso_commands[0x09] = create_command(0x09, "--set-camera-protocol", "set cameras protocol\n");
	imuso_commands[0x0A] = create_command(0x0A, "--get-camera-protocol", "get cameras protocol\n");
	imuso_commands[0x0B] = create_command(0x0B, "--set-outer-sw-value", "set switch value\n");
	imuso_commands[0x0C] = create_command(0x0C, "--get-current-light", "get current light value\n");
	imuso_commands[0x0D] = create_command(0x0D, "--reset", "reset to factory values\n");
	imuso_commands[0x0E] = create_command(0x0E, "--set-cur-mode", "force set cameras to current mode\n");
	imuso_commands[0x0F] = create_command(0x0F, "--get-light-duration", "get duration of impulse\n");
	imuso_commands[0x10] = create_command(0x10, "--set-light-duration", "set duration of impulse\n");

}


int findcommand(char *c)
{
	int i;
	for (i = 0; i<CMD_MAX; i++)
	{
		if (!strcmp(c, imuso_commands[i].text_cmd))
		{
			//       printf("finded %s\n",climat_commands[i].text_cmd);
			return i;
		}
	}
	return -1;
}

int imusoconn::imuso(int argc, char *argv[])
{

	init_commands_list();


	uint8_t* outbuf = (uint8_t *) malloc(7);
	uint8_t* command = (uint8_t *)malloc(4);

	if (argc <= 1){

		print_usage();
		return 0;

	}


	if (argc <= 2){
		if ((!strcmp(argv[1], "-?")) || (!strcmp(argv[1], "--help")) || (!strcmp(argv[1], "--usage"))){
			print_usage();
			return 0;
		}
	}

	char *portname = "/dev/ttyPTZ";
	int first_parameter = 2;

	if (argc >= 1) {

		char cc[3];
		memcpy(cc, argv[1], 3);
		cc[2] = 0;
		if (strcmp(cc, "--"))
		{
			portname = argv[1];
			first_parameter = 3;
		}
	}


	command[0] = CMD_GET_VERSION;

	int base = 16;

	if (argc>first_parameter - 1){

		char cc[3];
		memcpy(cc, argv[first_parameter - 1], 3);
		cc[2] = 0;

		if (!strcmp(cc, "--")){
			int cmd = findcommand(argv[first_parameter - 1]);
			if (cmd<0){
				printf("unknown command %s\n", argv[first_parameter - 1]);
				print_usage();
				return 0;
			}
			command[0] = cmd;

		}
		else command[0] = strtol(argv[first_parameter - 1], NULL, 16);
		//printf("%d %d \n",argc,first_parameter);
		if (argc>first_parameter){

			if ((!strcmp(argv[first_parameter], "--help")) || (!strcmp(argv[first_parameter], "-?"))){
				printf("%s %s", imuso_commands[command[0]].text_cmd, imuso_commands[command[0]].text_comment);
				return 0;
			}

			if ((!strcmp(argv[first_parameter], "--human")) || (!strcmp(argv[first_parameter], "-h"))){
				first_parameter = 4;
				process_kind = PROCESS_OUT_HUMAN;
				base = 16;
			}


		}
	}
	int i;
	for (i = 1; i <= 2; i++){
		command[i] = 0x00;
		if (argc >= i + first_parameter){
			//if ((command[0]==CMD_SET_POWER_LO_VOLT)||(command[0]==CMD_SET_POWER_HIGH_VOLT))
			command[i] = strtol(argv[i + first_parameter - 1], NULL, base);

		}
	}

	//command[2] = 0x00;
	//command[3] = 0x00;

	last_command = command[0];
	prepare_pelco_d_command(command, outbuf);


	int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		//        error_message ("error %d opening %s: %s", errno, portname, strerror (errno));
		printf("Error %d %s %s\n", errno, strerror(errno), portname);
		return 0;
	}

	set_interface_attribs(fd, B9600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking(fd, 0);                // set no blocking




	write(fd, outbuf, 8);
	free(command);
	free(outbuf);

	uint8_t buf;
	uint32_t timeout = 0;

	buffer_processed = 0;
	while (timeout<1){
		int n = read(fd, &buf, 1);

		if (n>0)
		{
			//printf(" %.2X",buf);
			processbyte(buf);
			timeout = 0;
		}
		else {
			timeout++;
			set_state(STATE_WAIT);
		}

	}

	close(fd);
	return !buffer_processed;
}