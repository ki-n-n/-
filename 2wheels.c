#ifdef __cplusplus
extern "C" {
void abort(void);
#endif
void main(void);
#ifdef __cplusplus
}
#endif

//インクルードファイル
#include<36064s.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "vs-bb020.h"
#include "memmap.h"

//グローバル変数

BYTE mode;						//動作モード（0：キャリブレーション、1：動作開始）
const BYTE MainCycle = 120;		//メインループ制御周波数
unsigned int main_counter = 0;	//スタート用カウンター
extern unsigned char memmap[MEMMAP_SIZE];

double out_L,out_R;					//出力値　演算結果

//倒立制御用パラメータ
volatile double angle;		//角度
volatile double dangle;		//角速度
volatile double d2angle;	//角加速度
volatile double z_L;		//移動距離
volatile double dz_L;		//移動速度
volatile double z_R;		//移動距離
volatile double dz_R;		//移動速度

//パラメータ算出用
volatile double old_dangle;	//角速度（1サイクル前の、角加速度算出用）
volatile double old_z_L;	//移動距離（1サイクル前の）
volatile double old_z_R;	//移動距離（1サイクル前の）

volatile int GyBuff[10];	//ジャイロセンサ用バッファ
volatile double GYCenter;	//ジャイロ原点（オフセット）

//前後移動用変数
int fwvale_L,fwvale_R;	//前後移動量
int amari_L,amari_R;	//加算時あまり用
extern int enc_L,enc_R;	//エンコーダ加算値

unsigned int vec_Bat[10];	//電池電圧用バッファ（Vbat/2を格納する）

//転倒検出用
int MaxPowerCntCW = 0;		//全開時間測定用（時計回り）
int MaxPowerCntCCW = 0;		//全開時間測定用（反時計回り）
int a;
int v;

//関数
extern void main(void);


//ジャイロセンサの値をバッファに格納
void setAdBuf(){
	int i;
	for(i = 9 ; i>0 ; i--){
		GyBuff[i] = GyBuff[i-1];
	}
	GyBuff[0] = GetGyro();
}

//ジャイロセンサの平均値を取得（原点を基準とした値で）
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
	temp -=(max + min);		//最大、最小の値は除去（急激なノイズの除去で動作を安定化）
	
	return (int)(temp/6.0 - GYCenter);
}

//制御パラメータのクリア
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

//ジャイロセンサのキャリブレーション用関数
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
	
	//グラフ表示がONならシリアル通信を許可
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
		SCI3_2.SCR3.BIT.TIE = 0;			//TDRE割り込み不可
	}
	
	ClearValue();	

	mode = MODE_RUN;
}

//左右モータの移動量を設定（パッド操作、ライントレースなど、移動する際に使用）
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

//動作モードを取得
BYTE getMode(){
	return mode;
}
void weels_main(void)
{
//変数
	int i=0;					//汎用
	UINT mtr[2] = {0,0};	//モータ出力	
	unsigned int *vecGain;	
	
 	vecGain = (unsigned int*)&memmap[GAIN_MAP];		//ゲイン用配列のアドレス設定
		
	mode = MODE_STOP;				//モードをストップに設定
	Init((BYTE)MainCycle);			//H8の初期化	
	InitSci3(CBR_115200,even,1);	//シリアルの初期化（ボーレート115200、偶数パリティ、ストップビット1bit)
	
	LED(1);	
	Mtr_Run_BB(0,0);		//モータ停止	
	Rd_MEMMAP();			//設定読込み	
	BuzzerSet(0x80,0x80);	//ブザーの設定

	//起動待ち
	for(i=0;i<10;i++)
		Sync();
	
	//ジャイロセンサ用バッファを埋める
	for(i = 0 ; i < 10 ; i++)
		setAdBuf();
		
//メインループ
	while(1){
		if((memmap[RUN_MODE]&0x80) || mode == MODE_STOP)
			CheckSci();		//シリアルチェック
			
		updatePAD();	//PADデータ更新
		
		//ジャイロセンサ値取得×7
		setAdBuf();	
		setAdBuf();
		Sync();
		setAdBuf();
		setAdBuf();
		setAdBuf();
		setAdBuf();
		setAdBuf();
		main_counter ++;
		
	//通信、キャリブレーションモード
		if(mode == MODE_STOP){  		
			main();			
		}
		
	//走行モード
		else if(mode == MODE_RUN){
			if(getSW()){		//動作中スイッチが押されたら
				ClearValue();	//パラメータ変数をクリアし一時停止
			}
			else if(main_counter > MainCycle*3){	//3秒後にメイン関数の動作開始（MainCycleが周波数なので...）
				main();		
			}
			
		//センサ値変換用定数
			#define conv_gy -0.120248368	//(-3.3/(4096.0*0.0067))
			#define conv_enc 0.9		//360.0/(5*4*40/2);
		
		//本体移動量加算
			enc_L += (fwvale_L + amari_L)/10;
			enc_R += (fwvale_R + amari_R)/10;
		
			amari_L = (fwvale_L + amari_L)%10;
			amari_R = (fwvale_R + amari_R)%10;
				
	//制御パラメータの算出
			old_dangle = dangle;
		//角速度の取得					
			dangle = (double)getGyAve()*conv_gy;	// 6.7mv/deg/sec, 3.3v = 4096	
		//角度の算出
			angle += dangle;
		//角加速度の算出	
			d2angle = dangle - old_dangle;
		
		//車輪位置の取得
			z_R = (double)enc_R*conv_enc;
			z_L = (double)enc_L*conv_enc;
		
		//電源電圧の取得	
			for(i = 9;i>0;i--){
				vec_Bat[i] = vec_Bat[i-1];
			}
			vec_Bat[0] = AdRead(7);
		
		//速度の算出
			dz_L = (z_L - old_z_L)*MainCycle;
			dz_R = (z_R - old_z_R)*MainCycle;
			old_z_L = z_L;
			old_z_R = z_R;
		
		//倒立制御
		{
			double ENC_GAIN = (double)vecGain[GAIN_EN_P]/1000.0;
			double dENC_GAIN = (double)vecGain[GAIN_EN_D]/1000.0;
			double ANGLE_GAIN = (double)vecGain[GAIN_GY_I]/1000.0;
			double dANGLE_GAIN = (double)vecGain[GAIN_GY_P]/1000.0;
			double d2ANGLE_GAIN = (double)vecGain[GAIN_GY_D]/1000.0;
			
		//電源電圧のフィードバック
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
			
		//モータ出力の算出
			out_L = ENC_GAIN*z_L 		//P 移動距離　（エンコーダ）
					+ dENC_GAIN*dz_L 	//D　速度　（エンコーダ）
					
					- 1.0 * ENC_GAIN * (z_L - z_R)
					- 0.5 * dENC_GAIN * (dz_L - dz_R)
				
				+ ANGLE_GAIN * angle  	 //P　角度（ジャイロ）
				+ dANGLE_GAIN * dangle 	//D　角速度（ジャイロ）
				+ d2ANGLE_GAIN * d2angle
			;
			
			out_R = ENC_GAIN*z_R 		//P 移動距離　（エンコーダ）
				+ dENC_GAIN*dz_R 	//D　速度　（エンコーダ）

				-1.0 * ENC_GAIN * (z_R - z_L)
				-0.5 * dENC_GAIN * (dz_R - dz_L)
								
				+ ANGLE_GAIN * angle  	//P　角度（ジャイロ）
				+ dANGLE_GAIN * dangle 	//D　角速度（ジャイロ）
				+ d2ANGLE_GAIN * d2angle
			;
			
			out_L *= (1.0+batvalue);
			out_R *= (1.0+batvalue);
		}
		
		//各モータ出力のオーバーフローチェック、出力設定
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
		//転倒検出（最高速が1秒続いたら）
	
			if(mtr[1] == 0x7FFF)
				MaxPowerCntCW++;
			else 
				MaxPowerCntCW = 0;
			if(mtr[1] == 0x8001)
				MaxPowerCntCCW++;
			else 
				MaxPowerCntCCW = 0;
				
		//転倒時の停止
			if(MaxPowerCntCW > 1*MainCycle || MaxPowerCntCCW > 1*MainCycle ){
				Mtr_Run_BB(0,0);
				Wait(1000);
				mode = MODE_STOP;
							
				ClearValue();
			}
		//モータへ出力
			else 
				Mtr_Run_BB(mtr[0],mtr[1]);
		}
	}
}

//ログ用シリアル出力
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
