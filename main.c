/*
 * main.c
 *
 *  Created on: Mar 19, 2013
 *      Author: nemo
 */
/*
 * 	SATOR
 *  AREPO
 *  TENET
 *  OPERA
 *  ROTAS
 */
#include "stdio.h"
#include "fcntl.h"
#include "unistd.h"
#include "termios.h"

#define	CMD_INIT				((unsigned char)0x7F)
#define	CMD_GET					((unsigned char)0x00)
#define	CMD_GET_BL_VER			((unsigned char)0x01)

#define	CMD_GET_ID				((unsigned char)0x02)
#define	CMD_READ_MEM			((unsigned char)0x11)
#define	CMD_GO					((unsigned char)0x21)
#define	CMD_WRITE_MEM			((unsigned char)0x31)
#define	CMD_ERASE				((unsigned char)0x43)
#define	CMD_EXT_ERASE			((unsigned char)0x44)
#define	CMD_WRITE_PROTECT		((unsigned char)0x63)
#define	CMD_WRITE_UNPROTECT		((unsigned char)0x73)
#define	CMD_READOUT_PROTECT		((unsigned char)0x82)
#define	CMD_READOUT_UNPROTECT	((unsigned char)0x92)

#define	CMD_GLOBAL_ERASE		((unsigned char)0xFF)

#define	ACK		0x79
#define	NACK	0x1F

#define	MAX_BYTES_TO_READ	256

int fd;
unsigned char errcode;

char get_command(void);

unsigned char send_command(char *, unsigned char);
void send_byte(unsigned char);
char recv_byte(unsigned char *);

void read_mem(unsigned char *, unsigned int addr, unsigned int num);
void write_mem(unsigned char *, unsigned int addr, unsigned int num);

char is_ack(void);

int main(void){
	int fil;
	int i;
	unsigned char bytes_to_read;
	unsigned char str[MAX_BYTES_TO_READ];
	unsigned char bootloader_version;
	unsigned char rpd;
	unsigned char rpe;

	unsigned int addr;
	unsigned int num;
	unsigned char checksum;

	unsigned char command;
	struct termios options;

	//fd = open("/dev/rfcomm0", O_RDWR);
	fd = open("/dev/ttyUSB0", O_RDWR/* | O_NOCTTY*/);

	if(fd == -1) return (1);

	fcntl(fd, F_SETFL, 0);

	tcgetattr(fd, &options);

	cfsetospeed(&options, B115200);

	options.c_cflag |= PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag |= CS8;
	options.c_lflag &= ~ICANON;

	options.c_cc[VMIN]  = 1;
	options.c_cc[VTIME] = 0;

	tcsetattr(fd, TCSANOW, &options);

	while((command = get_command()) != 'q'){
		switch(command){
			case 'i':	send_command("INIT", CMD_INIT);
						break;

			case 'g':	if(send_command("GET", CMD_GET) == ACK){
							printf("Bytes to read...");
							recv_byte(&bytes_to_read);
							printf("%d\n", bytes_to_read);

							printf("Reading bytes...");
							for(i=0; i<=bytes_to_read; i++)recv_byte(&str[i]);

							if(is_ack()){
								for(i=0; i<=bytes_to_read; i++) printf("%x ", str[i]);
								printf("\n");
							}
						}
						break;

			case 'v':	if(send_command("GET VER", CMD_GET_BL_VER) == ACK){
							printf("Params reading...");
							recv_byte(&bootloader_version);
							recv_byte(&rpd);
							recv_byte(&rpe);
							if(is_ack()){
								printf("%x %x %x\n", bootloader_version, rpd, rpe);
							}
						}
						break;

			case 'd':	if(send_command("GET ID", CMD_GET_ID) == ACK){
							printf("Bytes to read...");
							recv_byte(&bytes_to_read);
							printf("%d\n", bytes_to_read);

							printf("Reading bytes...");
							for(i=0 ;i<=bytes_to_read; i++){
								recv_byte(&str[i]);
							}

							if(is_ack()){
								for(i=0; i<=bytes_to_read; i++) printf("%x ", str[i]);
								printf("\n");
							}
						}
						break;

			case 'm':	printf("Enter start address (HEX)...");
						scanf("%x", &addr);
						printf("Enter num bytes (DEC)...");
						scanf("%d", &num);

						fil = open("readfile.txt", O_WRONLY);

						for(i=0; i<num/MAX_BYTES_TO_READ; i++){
							read_mem(str, addr + i*MAX_BYTES_TO_READ, MAX_BYTES_TO_READ);
							write(fil, str, MAX_BYTES_TO_READ);
							num -= MAX_BYTES_TO_READ;
						}

						if(num > 0){
							read_mem(str, addr + i*MAX_BYTES_TO_READ, num);
							write(fil, str, num);
						}

						close(fil);
						break;

			case 'o':	if(send_command("GO", CMD_GO) == ACK){
							printf("Enter start address (HEX)...");
							scanf("%x", &addr);

							printf("Sending start address (%08x)...", addr);
							send_byte((addr >> 24) & 0xFF);	checksum  =	(addr >> 24) & 0xFF;
							send_byte((addr >> 16) & 0xFF);	checksum ^=	(addr >> 16) & 0xFF;
							send_byte((addr >>  8) & 0xFF);	checksum ^=	(addr >>  8) & 0xFF;
							send_byte((addr >>  0) & 0xFF);	checksum ^=	(addr >>  0) & 0xFF;
							send_byte(checksum);
							if(is_ack()){
								printf("Getting acknowledge...");
								is_ack();
							}
						}
						break;

			case 'w':	printf("Enter start address (HEX)...");
						scanf("%x", &addr);

						fil = open("writefile.txt", O_RDONLY);
						num = lseek(fil, 0, SEEK_END);
						close(fil);
						fil = open("writefile.txt", O_WRONLY);

						for(i=0; i<num/MAX_BYTES_TO_READ; i++){
							read(fil, str, MAX_BYTES_TO_READ);
							write_mem(str, addr + i*MAX_BYTES_TO_READ, MAX_BYTES_TO_READ);
							num -= MAX_BYTES_TO_READ;
						}

						if(num > 0){
							read(fil, str, num);
							write_mem(str, addr + i*MAX_BYTES_TO_READ, MAX_BYTES_TO_READ);
						}

						close(fil);
						break;

			case 'e':	if(send_command("ERASE", CMD_ERASE) == ACK){
							printf("Enter amount of pages to be erased (DEC)...");
							scanf("%x", &num);

							if(num == 255){
								send_command("GLOBAL ERASE", CMD_GLOBAL_ERASE);
							}
							else{
								if(num > 0){
									for(i=0; i<num; i++){
										printf("Enter [%d] page number to be erased...", i);
										scanf("%d", &addr);
										str[i] = addr&0xFF;
									}

									printf("Erasing pages...");
									send_byte(num-1);			checksum  =	num-1;
									for(i=0; i<num; i++){
										send_byte(str[i]);		checksum ^= str[i];
									}
									send_byte(checksum);
									is_ack();
								}
								else{
									printf("Nothing to erase\n");
								}
							}
						}
						break;

			case 't':	if(send_command("WRITE PROTECT", CMD_WRITE_PROTECT) == ACK){
							printf("Enter amount of sectors to be protected (DEC)...");
							scanf("%x", &num);

							if(num > 0){
								for(i=0; i<num; i++){
									printf("Enter [%d] sector number to be protected...", i);
									scanf("%d", &addr);
									str[i] = addr&0xFF;
								}

								printf("Protecting sectors...");
								send_byte(num-1);			checksum  =	num-1;
								for(i=0; i<num; i++){
									send_byte(str[i]);		checksum ^= str[i];
								}
								send_byte(checksum);
								is_ack();
							}
							else{
								printf("Nothing to protect\n");
							}
						}
						break;

			case 'r':	send_command("WRITE UNPROTECT", CMD_WRITE_UNPROTECT);
						break;

			case 'a':	send_command("READOUT PROTECT", CMD_READOUT_PROTECT);
						break;

			case 'u':	send_command("READOUT UNPROTECT", CMD_READOUT_UNPROTECT);
						break;

			default:	printf("Not implemented yet\n");
						break;
		}
	}
	close(fd);

	printf("Bye.\n");
	return (0);
}

void read_mem(unsigned char *str, unsigned int fromaddr, unsigned int totalnum){
	int i, j;
	unsigned char checksum;

	union{
		unsigned int	ui;
		char			c[4];
	}addr;

	union{
		unsigned int	ui;
		char			c[4];
	}num;

	addr.ui = fromaddr;
	num.ui	= totalnum - 1;

	if(send_command("READ MEM", CMD_READ_MEM) == ACK){

		printf("Sending start address (%08x)...", addr.ui);
		send_byte(addr.c[3]);	checksum  = addr.c[3];
		send_byte(addr.c[2]);	checksum ^= addr.c[2];
		send_byte(addr.c[1]);	checksum ^= addr.c[1];
		send_byte(addr.c[0]);	checksum ^= addr.c[0];
		send_byte(checksum);
		if(is_ack()){
			printf("Sending num bytes (%d)...", num.ui);
			checksum = 0xFF;
			send_byte(num.c[0]);	checksum ^= num.c[0];
			send_byte(checksum);

			if(is_ack()){
				i=0;
				while(i<=num.c[0]){
					for(j=0; j<16; j++, i++){
						if(i <= num.c[0]){
							recv_byte(&str[i]);
							printf("%02x ", str[i]);
						}
						else{
							break;
						}
					}
					printf("\n");
				}
			}
		}
	}
}

void write_mem(unsigned char *str, unsigned int fromaddr, unsigned int totalnum){
	int i, j;
	char checksum;

	union{
		unsigned int	ui;
		char			c[4];
	}addr;
	union{
		unsigned int	ui;
		char			c[4];
	}num;

	addr.ui = fromaddr;
	num.ui	= totalnum-1;

	if(send_command("WRITE MEM", CMD_WRITE_MEM) == ACK){
		printf("Sending start address (%08x)...", addr.ui);
		send_byte(addr.c[3]);	checksum  = addr.c[3];
		send_byte(addr.c[2]);	checksum ^= addr.c[2];
		send_byte(addr.c[1]);	checksum ^= addr.c[1];
		send_byte(addr.c[0]);	checksum ^= addr.c[0];
		send_byte(checksum);
		if(is_ack()){

			printf("Sending num bytes (%d)...", num.ui);
			checksum = 0xFF;
			send_byte(num.c[0]);	checksum ^= num.c[0];
			send_byte(checksum);

			if(is_ack()){
				i=0;
				checksum = 0;
				printf("Sending bytes ...");
				while(i<=num.c[0]){
					for(j=0; j<16; j++, i++){
						if(i <= num.c[0]) send_byte(str[i]);	checksum ^= str[i];
					}
					printf("\n");
				}
				send_byte(checksum);
				is_ack();
			}
		}
	}
}

void send_byte(unsigned char byte){
	write(fd, &byte, 1);
	tcdrain(fd);
}

unsigned char send_command(char *title, unsigned char byte){
	printf("Sending %s command...", title);

	send_byte(byte);
	if(byte != 0x7F) send_byte(byte^0xFF);

	if(is_ack()){
		return (ACK);
	}

	return(NACK);
}

char recv_byte(unsigned char *byte){
	read(fd, byte, 1);
	return (byte[0]);
}

char is_ack(void){
	if(recv_byte(&errcode) == ACK){
		printf("ACK\n");
		return (1);
	}
	else{
		printf("NACK\n");
		return (0);
	}
}

char get_command(void){
	char a;
	printf("\n\n");
	printf("[q]quit             [i]nit              [g]et               get [v]ersion\n");
	printf("get i[d]            read [m]emory       g[o]                [w]rite memory\n");
	printf("wri[t]e protect     write unp[r]otect   re[a]d protect      read [u]nprotect\n");

	printf("Select command: ");
	scanf("%c", &a);

	return (a|0x20);
}
