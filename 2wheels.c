#ifdef __cplusplus
extern "C" {
void abort(void);
#endif
void main(void);
#ifdef __cplusplus
}
#endif

//�C���N���[�h�t�@�C��
#include<36064s.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "vs-bb020.h"
#include "memmap.h"

//�O���[�o���ϐ�

BYTE mode;						//���샂�[�h�i0�F�L�����u���[�V�����A1�F����J�n�j
const BYTE MainCycle = 120;		//���C�����[�v������g��
unsigned int main_counter = 0;	//�X�^�[�g�p�J�E���^�[
extern unsigned char memmap[MEMMAP_SIZE];

double out_L,out_R;					//�o�͒l�@���Z����

//�|������p�p�����[�^
volatile double angle;		//�p�x
volatile double dangle;		//�p���x
volatile double d2angle;	//�p�����x
volatile double z_L;		//�ړ�����
volatile double dz_L;		//�ړ����x
volatile double z_R;		//�ړ�����
volatile double dz_R;		//�ړ����x

//�p�����[�^�Z�o�p
volatile double old_dangle;	//�p���x�i1�T�C�N���O�́A�p�����x�Z�o�p�j
volatile double old_z_L;	//�ړ������i1�T�C�N���O�́j
volatile double old_z_R;	//�ړ������i1�T�C�N���O�́j

volatile int GyBuff[10];	//�W���C���Z���T�p�o�b�t�@
volatile double GYCenter;	//�W���C�����_�i�I�t�Z�b�g�j

//�O��ړ��p�ϐ�
int fwvale_L,fwvale_R;	//�O��ړ���
int amari_L,amari_R;	//���Z�����܂�p
extern int enc_L,enc_R;	//�G���R�[�_���Z�l

unsigned int vec_Bat[10];	//�d�r�d���p�o�b�t�@�iVbat/2���i�[����j

//�]�|���o�p
int MaxPowerCntCW = 0;		//�S�J���ԑ���p�i���v���j
int MaxPowerCntCCW = 0;		//�S�J���ԑ���p�i�����v���j
int a;
int v;

//�֐�
extern void main(void);


//�W���C���Z���T�̒l���o�b�t�@�Ɋi�[
void setAdBuf(){
	int i;
	for(i = 9 ; i>0 ; i--){
		GyBuff[i] = GyBuff[i-1];
	}
	GyBuff[0] = GetGyro();
}

//�W���C���Z���T�̕��ϒl���擾�i���_����Ƃ����l�Łj
int getGyAve(){
	int i,max = -5000 ,min = 5000;
	double temp = 0;
	
	for(i=0 ;i<8;i++){
		temp+=(double)GyBuff[i];
		if(GyBuff[i] > max)
			max = GyBuff[i];
		if(GyBuff[i] < min)
			min = GyBuff[i];
	}
	temp -=(max + min);		//�ő�A�ŏ��̒l�͏����i�}���ȃm�C�Y�̏����œ�������艻�j
	
	return (int)(temp/6.0 - GYCenter);
}

//����p�����[�^�̃N���A
void ClearValue(){
	ClearZ();
	main_counter = 0;
	angle = 0.0;
	dangle = 0.0;
	d2angle = 0.0;
	old_dangle = 0.0;
	z_L = 0.0;
	old_z_L = 0.0;
	z_R = 0.0;
	old_z_R = 0.0;
	dangle = 0.0;
	dz_L = 0.0;
	dz_R = 0.0;
	fwvale_R = 0;
	fwvale_L = 0;
	amari_L = 0;
	amari_R = 0;
	ClearZ();	
}

//�W���C���Z���T�̃L�����u���[�V�����p�֐�
void calibration(){
	int n;
	double tempValue = 0.0;
	
	BuzzerStop();
	Sync();
	
	for(n =0;n<100;n++){
		tempValue += (double)GetGyro();
		Sync();
	}
	GYCenter = (tempValue/100.0);
	tempValue = 0.0;
	memset(GyBuff , GYCenter,sizeof(GyBuff));
	
	//�O���t�\����ON�Ȃ�V���A���ʐM������
	if(memmap[RUN_MODE]&0x80){
		if(memmap[SCI_READ] == 0)
			SCI3.SCR3.BIT.RIE = 1;
		else
			SCI3_2.SCR3.BIT.RIE = 1;
	}
	else{		
		SCI3.SCR3.BIT.RIE = 0;
		SCI3_2.SCR3.BIT.RIE = 0;
		SCI3_2.SCR3.BIT.TEIE = 0;
		SCI3_2.SCR3.BIT.TIE = 0;			//TDRE���荞�ݕs��
	}
	
	ClearValue();	

	mode = MODE_RUN;
}

//���E���[�^�̈ړ��ʂ�ݒ�i�p�b�h����A���C���g���[�X�ȂǁA�ړ�����ۂɎg�p�j
void setMtrSpeed(int L , int R){
	#define SPD_BUF 4 
	static int L_buff[SPD_BUF],R_buff[SPD_BUF];
	BYTE i;
	
	for(i = SPD_BUF - 1 ; i > 0 ; i--){
		L_buff[i] = L_buff[i - 1];
		R_buff[i] = R_buff[i - 1];                                                                                        
	a=AdRead(5);
	}
	
	L_buff[0] = L;
	R_buff[0] = R;
		
	if(a>750){
		v = 80;	
	}
	
	else{
		v = 0;		
	}
	
	fwvale_R = -(L_buff[0] + (int)(0.0*(double)(L_buff[0] - L_buff[1]))+v);
	fwvale_L = -(R_buff[0] + (int)(0.0*(double)(R_buff[0] - R_buff[1]))+v);  
		
	
}

//���샂�[�h���擾
BYTE getMode(){
	return mode;
}
void weels_main(void)
{
//�ϐ�
	int i=0;					//�ėp
	UINT mtr[2] = {0,0};	//���[�^�o��	
	unsigned int *vecGain;	
	
 	vecGain = (unsigned int*)&memmap[GAIN_MAP];		//�Q�C���p�z��̃A�h���X�ݒ�
		
	mode = MODE_STOP;				//���[�h���X�g�b�v�ɐݒ�
	Init((BYTE)MainCycle);			//H8�̏�����	
	InitSci3(CBR_115200,even,1);	//�V���A���̏������i�{�[���[�g115200�A�����p���e�B�A�X�g�b�v�r�b�g1bit)
	
	LED(1);	
	Mtr_Run_BB(0,0);		//���[�^��~	
	Rd_MEMMAP();			//�ݒ�Ǎ���	
	BuzzerSet(0x80,0x80);	//�u�U�[�̐ݒ�

	//�N���҂�
	for(i=0;i<10;i++)
		Sync();
	
	//�W���C���Z���T�p�o�b�t�@�𖄂߂�
	for(i = 0 ; i < 10 ; i++)
		setAdBuf();
		
//���C�����[�v
	while(1){
		if((memmap[RUN_MODE]&0x80) || mode == MODE_STOP)
			CheckSci();		//�V���A���`�F�b�N
			
		updatePAD();	//PAD�f�[�^�X�V
		
		//�W���C���Z���T�l�擾�~7
		setAdBuf();	
		setAdBuf();
		Sync();
		setAdBuf();
		setAdBuf();
		setAdBuf();
		setAdBuf();
		setAdBuf();
		main_counter ++;
		
	//�ʐM�A�L�����u���[�V�������[�h
		if(mode == MODE_STOP){  		
			main();			
		}
		
	//���s���[�h
		else if(mode == MODE_RUN){
			if(getSW()){		//���쒆�X�C�b�`�������ꂽ��
				ClearValue();	//�p�����[�^�ϐ����N���A���ꎞ��~
			}
			else if(main_counter > MainCycle*3){	//3�b��Ƀ��C���֐��̓���J�n�iMainCycle�����g���Ȃ̂�...�j
				main();		
			}
			
		//�Z���T�l�ϊ��p�萔
			#define conv_gy -0.120248368	//(-3.3/(4096.0*0.0067))
			#define conv_enc 0.9		//360.0/(5*4*40/2);
		
		//�{�̈ړ��ʉ��Z
			enc_L += (fwvale_L + amari_L)/10;
			enc_R += (fwvale_R + amari_R)/10;
		
			amari_L = (fwvale_L + amari_L)%10;
			amari_R = (fwvale_R + amari_R)%10;
				
	//����p�����[�^�̎Z�o
			old_dangle = dangle;
		//�p���x�̎擾					
			dangle = (double)getGyAve()*conv_gy;	// 6.7mv/deg/sec, 3.3v = 4096	
		//�p�x�̎Z�o
			angle += dangle;
		//�p�����x�̎Z�o	
			d2angle = dangle - old_dangle;
		
		//�ԗֈʒu�̎擾
			z_R = (double)enc_R*conv_enc;
			z_L = (double)enc_L*conv_enc;
		
		//�d���d���̎擾	
			for(i = 9;i>0;i--){
				vec_Bat[i] = vec_Bat[i-1];
			}
			vec_Bat[0] = AdRead(7);
		
		//���x�̎Z�o
			dz_L = (z_L - old_z_L)*MainCycle;
			dz_R = (z_R - old_z_R)*MainCycle;
			old_z_L = z_L;
			old_z_R = z_R;
		
		//�|������
		{
			double ENC_GAIN = (double)vecGain[GAIN_EN_P]/1000.0;
			double dENC_GAIN = (double)vecGain[GAIN_EN_D]/1000.0;
			double ANGLE_GAIN = (double)vecGain[GAIN_GY_I]/1000.0;
			double dANGLE_GAIN = (double)vecGain[GAIN_GY_P]/1000.0;
			double d2ANGLE_GAIN = (double)vecGain[GAIN_GY_D]/1000.0;
			
		//�d���d���̃t�B�[�h�o�b�N
			double batvalue = 0.0;
			int bat_min = 1023,bat_max = 0;
			
			for(i = 0;i<10;i++){
				batvalue += vec_Bat[i];
				if(bat_min > vec_Bat[i])
					bat_min = vec_Bat[i];
				if(bat_max < vec_Bat[i])
					bat_max = vec_Bat[i];
			}
			
			batvalue -= (bat_max + bat_min);
 			batvalue = (batvalue / (1024.0 * 8.0) ) *  3.3 ;
			batvalue = (1.1 - batvalue)*2.5;
			
			if(batvalue > 0.0)
				batvalue = 0.0;
			else if(batvalue < -0.5)
				batvalue = -0.5;
			
		//���[�^�o�͂̎Z�o
			out_L = ENC_GAIN*z_L 		//P �ړ������@�i�G���R�[�_�j
					+ dENC_GAIN*dz_L 	//D�@���x�@�i�G���R�[�_�j
					
					- 1.0 * ENC_GAIN * (z_L - z_R)
					- 0.5 * dENC_GAIN * (dz_L - dz_R)
				
				+ ANGLE_GAIN * angle  	 //P�@�p�x�i�W���C���j
				+ dANGLE_GAIN * dangle 	//D�@�p���x�i�W���C���j
				+ d2ANGLE_GAIN * d2angle
			;
			
			out_R = ENC_GAIN*z_R 		//P �ړ������@�i�G���R�[�_�j
				+ dENC_GAIN*dz_R 	//D�@���x�@�i�G���R�[�_�j

				-1.0 * ENC_GAIN * (z_R - z_L)
				-0.5 * dENC_GAIN * (dz_R - dz_L)
								
				+ ANGLE_GAIN * angle  	//P�@�p�x�i�W���C���j
				+ dANGLE_GAIN * dangle 	//D�@�p���x�i�W���C���j
				+ d2ANGLE_GAIN * d2angle
			;
			
			out_L *= (1.0+batvalue);
			out_R *= (1.0+batvalue);
		}
		
		//�e���[�^�o�͂̃I�[�o�[�t���[�`�F�b�N�A�o�͐ݒ�
			if(out_L>=0.0){
				long OutPutTemp = 0;
				OutPutTemp = (long)(out_L * 256);
				if(OutPutTemp>0x7FFF)
					OutPutTemp = 0x7FFF;
				mtr[0] = (UINT)(0x10000 - OutPutTemp);
			}
			else{
				long OutPutTemp = 0;
				OutPutTemp = -(long)(out_L * 256);
				if(OutPutTemp > 0x7FFF)
					OutPutTemp = 0x7FFF;
				mtr[0] = (UINT)OutPutTemp;
			}
			if(out_R>=0.0){
				long OutPutTemp = 0;
				OutPutTemp = (long)(out_R * 256);
				if(OutPutTemp>0x7FFF)
					OutPutTemp = 0x7FFF;
				mtr[1] = (UINT)(0x10000 - OutPutTemp);
			}
			else{
				long OutPutTemp = 0;
				OutPutTemp = -(long)(out_R * 256);
				if(OutPutTemp > 0x7FFF)
					OutPutTemp = 0x7FFF;
				mtr[1] = (UINT)OutPutTemp;
			}
		//�]�|���o�i�ō�����1�b��������j
	
			if(mtr[1] == 0x7FFF)
				MaxPowerCntCW++;
			else 
				MaxPowerCntCW = 0;
			if(mtr[1] == 0x8001)
				MaxPowerCntCCW++;
			else 
				MaxPowerCntCCW = 0;
				
		//�]�|���̒�~
			if(MaxPowerCntCW > 1*MainCycle || MaxPowerCntCCW > 1*MainCycle ){
				Mtr_Run_BB(0,0);
				Wait(1000);
				mode = MODE_STOP;
							
				ClearValue();
			}
		//���[�^�֏o��
			else 
				Mtr_Run_BB(mtr[0],mtr[1]);
		}
	}
}

//���O�p�V���A���o��
void send_data(char str){
	char i,j;
	char txt[48];
	char txt_size;
	txt_size = 0;
	if(str == 'g'){
		txt[txt_size] = 'g';
			txt_size++;
		
		for(j = 0;j < 3;j++){
			char* f_ptr;
			if(j == 0)
				f_ptr = (char*)&d2angle;
			else if(j == 1)
				f_ptr = (char*)&dangle;
			else
				f_ptr = (char*)&angle;
			
			txt[txt_size] = ':';
			txt_size++;
			for(i = 0;i<4;i++){
				txt[txt_size] = f_ptr[i];
				txt_size++;
			}
		}
	}
	else if(str == 'l'){
		txt[txt_size] = 'l';
			txt_size++;
		
		{
			char* f_ptr;
			f_ptr = (char*)&z_L;
			
			txt[txt_size] = ':';
			txt_size++;
			for(i = 0;i<4;i++){
				txt[txt_size] = f_ptr[i];
				txt_size++;
			}

			f_ptr = (char*)&dz_L;
			txt[txt_size] = ':';
			txt_size++;
			for(i = 0;i<4;i++){
				txt[txt_size] = f_ptr[i];
				txt_size++;
			}
		}
	}
	else if(str == 'm'){
		txt[txt_size] = 'm';
		txt_size++;
		txt[txt_size] = ':';
		txt_size++;
		
		{
			char* f_ptr;
			f_ptr = (char*)&z_R;
			
			for(i = 0;i<4;i++){
				txt[txt_size] = f_ptr[i];
				txt_size++;
			}

			txt[txt_size] = ':';
			txt_size++;
		
			f_ptr = (char*)&dz_R;
			for(i = 0;i<4;i++){
				txt[txt_size] = f_ptr[i];
				txt_size++;
			}
		}
	}

	else if(str == 'o'){
		txt[txt_size] = 'o';
		txt_size++;
		txt[txt_size] = ':';
		txt_size++;
		
		{
			char* f_ptr;
			f_ptr = (char*)&out_L;
			
			for(i = 0;i<4;i++){
				txt[txt_size] = f_ptr[i];
				txt_size++;
			}

			txt[txt_size] = ':';
			txt_size++;
		
			f_ptr = (char*)&out_R;
			for(i = 0;i<4;i++){
				txt[txt_size] = f_ptr[i];
				txt_size++;
			}
		}
	}
	if(txt_size > 0){
		txt[txt_size] = '\r';
		txt_size++;
		txt[txt_size] = '\n';
		txt_size++;
		txt[txt_size] = '\0';
		SciStrTx(txt,txt_size);
	}
}



#ifdef __cplusplus
void abort(void)
{
	
}
#endif
