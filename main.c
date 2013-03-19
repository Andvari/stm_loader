/*
 * main.c
 *
 *  Created on: Mar 19, 2013
 *      Author: nemo
 */

#include "stdio.h"
#include "fcntl.h"
#include "unistd.h"
#include "termios.h"

#define	CMD_INIT			((unsigned char)0x7F)
#define	CMD_GET				((unsigned char)0x00)
#define	CMD_GET_BL_VER		((unsigned char)0x01)

#define	CMD_GET_ID			((unsigned char)0x02)
#define	CMD_READ_MEM		((unsigned char)0x11)
#define	CMD_GO				((unsigned char)0x21)
#define	CMD_WRITE_MEM		((unsigned char)0x31)
#define	CMD_ERASE			((unsigned char)0x43)
#define	CMD_EXT_ERASE		((unsigned char)0x44)
#define	CMD_WRITE_PROTECT	((unsigned char)0x63)
#define	CMD_WRITE_UNPROTECT	((unsigned char)0x73)
#define	CMD_READ_PROTECT	((unsigned char)0x82)
#define	CMD_READ_UNPROTECT	((unsigned char)0x92)


#define	ACK		0x79
#define	NACK	0x1F

#define	MAX_BYTES_TO_READ	256

int fd;
unsigned char errcode;

void send_command(unsigned char);
void send_byte(unsigned char);
void recv_byte(unsigned char *);
void get_params(int *, int *);

void init(void);
void get(void);
void get_bootloader_version(void);
void get_id(void);
void read_mem(void);
void go(void);



char checksum(int);

void get_ack_nack(void);

int main(void){
	int command;
	struct termios options;
	errcode = ACK;

	//fd = open("/dev/rfcomm0", O_RDWR);
	fd = open("/dev/ttyUSB1", O_RDWR/* | O_NOCTTY*/);

	fcntl(fd, F_SETFL, 0);

	tcgetattr(fd, &options);

	cfsetospeed(&options, B4800);

	options.c_cflag |= PARENB;
	options.c_cflag &= ~CSTOPB;
	//options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	tcsetattr(fd, TCSANOW, &options);


	printf("fd: %08x\n", fd);

	command = -1;
	while(command != 0){
		printf("0 - Quit\n");
		printf("1 - Init\n");
		printf("2 - Get\n");
		printf("3 - Get bootloader version\n");
		printf("4 - Get ID\n");
		printf("5 - Read memory\n");
		printf("6 - Go\n");
		printf("100 - Unhang\n");
		printf("Select command\n");
		scanf("%d", &command);

		printf("command: %d\n", command);
		switch(command){
			case 1:		init();
						break;
			case 2:		get();
						break;
			case 3:		get_bootloader_version();
						break;
			case 4:		get_id();
						break;
			case 5:		read_mem();
						break;
			case 6:		go();
						break;
			case 100:	send_byte(0x7F);
						break;
			default:	break;
		}
	}
	close(fd);

	return (0);
}

void init(void){
	errcode = NACK;
	printf("Sending init...");
	while(errcode != ACK){
		send_command(CMD_INIT);
	}
}

void get(void){
	int i;
	unsigned char bytes_to_read;
	unsigned char str[MAX_BYTES_TO_READ];

	printf("Sending GET command...");
	send_command(CMD_GET);

	if(errcode == ACK){
		printf("Bytes to read...");
		recv_byte(&bytes_to_read);
		printf("%d\n", bytes_to_read);
		printf("Reading bytes...");

		for(i=0; i<=bytes_to_read; i++)recv_byte(&str[i]);
		get_ack_nack();

		if(errcode == ACK){
			for(i=0; i<=bytes_to_read; i++) printf("%x ", str[i]);
			printf("\n");
		}
	}

}

void get_bootloader_version(void){
	unsigned char bootloader_version;
	unsigned char rpd;
	unsigned char rpe;
	printf("Sending GET BOOTLOADER VER command...");
	send_command(CMD_GET_BL_VER);
	recv_byte(&bootloader_version);
	recv_byte(&rpd);
	recv_byte(&rpe);
	get_ack_nack();
	if(errcode == ACK){
		printf("%x %x %x\n", bootloader_version, rpd, rpe);
	}
}

void get_id(void){
	int i;
	unsigned char str[MAX_BYTES_TO_READ];
	unsigned char bytes_to_read;
	printf("Sending GET ID command...");
	send_command(CMD_GET_ID);
	if(errcode == ACK){
		printf("Bytes to read...");
		recv_byte(&bytes_to_read);
		printf("%d\n", bytes_to_read);
		printf("Reading bytes...");
		for(i=0 ;i<=bytes_to_read; i++){
			recv_byte(&str[i]);
		}
		get_ack_nack();
		if(errcode == ACK){
			for(i=0; i<=bytes_to_read; i++) printf("%x ", str[i]);
			printf("\n");
		}
	}
}

void read_mem(void){
	int addr;
	int num;
	int i;
	unsigned char str[MAX_BYTES_TO_READ];

	get_params(&addr, &num);

	printf("Sending READ MEM command...");
	send_command(CMD_READ_MEM);
	if(errcode == ACK){
		printf("Sending start address (%08x)...", addr);
		send_byte((addr >> 24)&0xFF);
		send_byte((addr >> 16)&0xFF);
		send_byte((addr >>  8)&0xFF);
		send_byte((addr >>  0)&0xFF);
		send_byte(checksum(addr));
		get_ack_nack();
		if(errcode == ACK){
			printf("Sending num bytes (%d)...", num);
			send_byte(num&0xFF);
			send_byte(checksum((num&0xFF)^0xFF));
			get_ack_nack();
			if(errcode == ACK){
				for(i=0; i<=num; i++){
					recv_byte(&str[i]);
					printf("%02x ", str[i]);
				}
				printf("\n");
			}
		}
	}
}

void go(void){
	int addr;

	printf("Enter start address\n");
	scanf("%x", &addr);

	printf("Sending GO command...");
	send_command(CMD_GO);
	if(errcode == ACK){
		printf("Sending start address (%08x)...", addr);
		send_byte((addr >> 24)&0xFF);
		send_byte((addr >> 16)&0xFF);
		send_byte((addr >>  8)&0xFF);
		send_byte((addr >>  0)&0xFF);
		send_byte(checksum(addr));
		get_ack_nack();
		if(errcode == ACK){
			get_ack_nack();
			if(errcode == ACK){
				printf("OK\n");
			}
		}
	}
}


void send_byte(unsigned char byte){
	int i;
	int a;
	a = write(fd, &byte, 1);
	for(i=0; i<500000;i++)a++;
}

void send_command(unsigned char byte){
	send_byte(byte);
	if(byte != 0x7F){
		send_byte(byte^0xFF);
	}
	get_ack_nack();
}

void recv_byte(unsigned char *byte){
	int i;
	int a;
	a = read(fd, byte, 1);
	for(i=0; i<1000000;i++)a++;
}

void get_ack_nack(void){
	int i;
	int a;
	recv_byte(&errcode);
	//printf("errcode: %x\n", errcode);
}

void get_params(int *addr, int *num){

	printf("Enter start address\n");
	scanf("%x", addr);
	printf("Enter num bytes\n");
	scanf("%d", num);

	return;
}

char checksum(int val){
	char chksum;
	chksum = (((val >> 24)&0xFF)^(( val >> 16)&0xFF)^((val >> 8)&0xFF)^((val >> 0)&0xFF));

	printf("Checksum: %x\n", chksum);

	return (chksum);
}
