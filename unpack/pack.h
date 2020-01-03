#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/*
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
*/
//#include "bootflash.h"

#define UPFILE_NAME			"digicap.dav"	
#define MAX_FILE_NUM		32
//#define FLASH_SECTOR_SIZE	(64*1024)  /*64KB*/

#define IGNORE_VALUE8		0xff
#define IGNORE_VALUE32		0xffffffff
#define CFG_MAGIC			0x484b5753 
#define RESERVED_FEA_NUMS	16

typedef struct{			/* 44 bytes */
	char fileName[32];	/* �ļ��� */
	int	startOffset;	/* ��ʼλ�� */
	int	fileLen;		/* �ļ����� */
	int  checkSum;	/* У��� */
}UPGRADE_FILE_HEADER;

typedef struct {	/* 64 bytes */
	unsigned int	magic_number;			/* 0x484b5753 */
	unsigned int	header_check_sum;		/* �ļ�ͷУ��� */
	unsigned int	header_length;			/* �ļ�ͷ���� */
	unsigned int	file_nums;			/* �ļ����� */
	unsigned int	language;			/* ���� */
	unsigned int	device_class;			/* 1 �C DS9000 DVR */
	unsigned int	oemCode;			/* 1 �C hikvision  */
	unsigned char	res_feature[RESERVED_FEA_NUMS];	/* �����ֶ� */
	unsigned char	res[20];
	UPGRADE_FILE_HEADER  fileHeader[];
//	UPGRADE_FILE_HEADER  fileHeader[2];
//	UPGRADE_FILE_HEADER*  fileHeader;
}FIRMWARE_HEADER;
typedef struct {	/* 64 bytes */
	unsigned int	magic_number;			/* 0x484b5753 */
	unsigned int	header_check_sum;		/* �ļ�ͷУ��� */
	unsigned int	header_length;			/* �ļ�ͷ���� */
	unsigned int	file_nums;			/* �ļ����� */
	unsigned int	language;			/* ���� */
	unsigned int	device_class;			/* 1 �C DS9000 DVR */
	unsigned int	oemCode;			/* 1 �C hikvision  */
	unsigned char	res_feature[RESERVED_FEA_NUMS];	/* �����ֶ� */
	unsigned char	res[20];
//	UPGRADE_FILE_HEADER  fileHeader[0];
//	UPGRADE_FILE_HEADER*  fileHeader;
}FIRMWARE_HEADER2;

int main_parse_flash_pcm_info(void);

//void get_pcm_data_by_name(char* fileName);
unsigned char get_pcm_data_by_name(char* filename,unsigned char**pcm_flash_address,unsigned int* pcm_flash_len);

