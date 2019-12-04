#include <opencv2/opencv.hpp>
#include <SharedMemGrabber.h>
#include <INIReader.h>
#include <unistd.h>
#include <Tcl.h>
#include <chrono>
#include <inttypes.h>
#include <ctime>
#include <imusoconn.h>

using namespace cv;
using namespace std;

typedef struct _TraffickLight
{
	bool E1;				//показатель работы фазы
	bool E1Tcl;				//данные с TCL
	int E1TclChannel;		//канал детектора TCL на который посылаем данные
	int E1C;				//значение красной состовляющей
	int E1B_D;				//значение белой состовляющей DeadZone
	int E1B;				//значение белой состовляющей зоны интереса
	int E1V;				//значение размера овала максимально большой зоны
	int E1Color;			//цвет секции
	int E1PixelColor;		//числовое представление цвета для детекции пикселов
	float_t E1CAverage;		//среднее значение цветовой составляющей за 120 кадров без учёта нулевых значений
	Scalar E1ScalarColor;   //цвет секции в Scalar
	RotatedRect E1RR;		//вращающийся прямоугольник	
	Point E1V_CENTER;		//центр максимально большой зоны  			

	bool E2;
	bool E2Tcl;
	int E2TclChannel;
	int E2C;
	int E2B_D;
	int E2B;
	int E2V;
	int E2Color;
	int E2PixelColor;		//числовое представление цвета для детекции пикселов
	float_t E2CAverage;
	Scalar E2ScalarColor;
	RotatedRect E2RR;
	Point E2V_CENTER;										  	

	bool E3;
	bool E3Tcl;
	int E3TclChannel;
	int E3C;
	int E3B_D;
	int E3B;
	int E3V;
	int E3Color;
	int E3PixelColor;		//числовое представление цвета для детекции пикселов
	float_t E3CAverage;
	Scalar E3ScalarColor;
	RotatedRect E3RR;
	Point E3V_CENTER;										  	

	bool E4;
	bool E4Tcl;
	int E4TclChannel;
	int E4C;
	int E4B_D;
	int E4B;
	int E4V;
	int E4Color;
	int E4PixelColor;		//числовое представление цвета для детекции пикселов
	float_t E4CAverage;
	Scalar E4ScalarColor;
	RotatedRect E4RR;
	Point E4V_CENTER;										  	

	bool E5;
	bool E5Tcl;
	int E5TclChannel;
	int E5C;
	int E5B_D;
	int E5B;
	int E5V;
	int E5Color;
	int E5PixelColor;		//числовое представление цвета для детекции пикселов
	float_t E5CAverage;
	Scalar E5ScalarColor;
	RotatedRect E5RR;
	Point E5V_CENTER;										  	

	int RoiGrayGain;
	int UseErosion;
	int MaximumColorPixInMem;
} TraffickLight;

typedef struct _TraffickColorAverage
{
	int C;
	float Average;		//среднее значение цветовой составляющей за 120 кадров без учёта нулевых значений
	float Average2;		//среднее значение cредних значений цветовой составляющей за 120 кадров без учёта нулевых значений
}
TraffickColorAverage;

vector<TraffickColorAverage>  E1CAverage;
vector<TraffickColorAverage>  E2CAverage;
vector<TraffickColorAverage>  E3CAverage;
vector<TraffickColorAverage>  E4CAverage;
vector<TraffickColorAverage>  E5CAverage;
vector<Rect> ROI;
vector<Rect> ROItest;
vector<TraffickLight> TraffMem;
Mat Color288x50;
Mat empty;
Mat RoiHSV;
Mat RoiGray;
Mat RoiContours;
Mat joinImg;
Mat TmpGray;
Mat TmpColor;
Mat SubImageColorPic = Mat(288, 50, CV_8UC3);

int MaxColorAverage = 0;
int ShowGraph;						//Глобальный параметр выбора данных графика 0 - цвет, 1- зона детекции 2 - выходные данные
int pict_height = 288;				//Разрешение входного изображения по высоте
int pict_width;						//Разрешение входного изображения по ширене
int channelSharedMemGrabber;	    //Номер канала видеозахвата
int gaussBlur;						//Степень первоначального размытия RoiHSV перед преобразованием в чб
int RoiGrayGain;					//Степень усилиения/гашения чб перед детекцией 
int DeadZone;						//Размер квадрата мёртвой зоны в углах зон. В них не должно быть большой яркосной состовляющей
int thresh;							//Чуствительность детектора ярокстных вспышек области
Point PointJitterTrust = Point(3,3);
int UseErosion;						//Сужение больших яркосных пятен
int E1V, E2V, E3V, E4V, E5V = 0;
Point E1V_CENTER, E2V_CENTER, E3V_CENTER, E4V_CENTER, E5V_CENTER;

int E1CStartLag, E2CStartLag, E3CStartLag, E4CStartLag, E5CStartLag = 0; 	// количество кадров отличающихся от данных с TCL в начале фазы
int E1CEndLag, E2CEndLag, E3CEndLag, E4CEndLag, E5CEndLag = 0;				// количество кадров отличающихся от данных с TCL в конце фазы
int E1COutLag, E2COutLag, E3COutLag, E4COutLag, E5COutLag = 0;				// количество ЛОЖНЫХ сработок
int E1CInLag, E2CInLag, E3CInLag, E4CInLag, E5CInLag = 0;					// количество обрывов 
int E1TLCState, E2TLCState, E3TLCState, E4TLCState, E5TLCState = 0;			// триггер состояний
int E1CStartLag_tmp, E2CStartLag_tmp, E3CStartLag_tmp, E4CStartLag_tmp, E5CStartLag_tmp = 0; 
int E1CEndLag_tmp, E2CEndLag_tmp, E3CEndLag_tmp, E4CEndLag_tmp, E5CEndLag_tmp = 0;
int E1TCLPhaseCount, E2TCLPhaseCount, E3TCLPhaseCount, E4TCLPhaseCount, E5TCLPhaseCount = 0;

void DrawDataToDebugPic()
{
	int E1C = 0;
	int E2C = 0;
	int E3C = 0;
	int E4C = 0;
	int E5C = 0;								  

	int E1C_1 = 0;
	int E2C_1 = 0;
	int E3C_1 = 0;
	int E4C_1 = 0;
	int E5C_1 = 0;	
	
	int E1Tcl, E2Tcl, E3Tcl, E4Tcl, E5Tcl = 0;
	int E1Tcl_1, E2Tcl_1, E3Tcl_1, E4Tcl_1, E5Tcl_1 = 0;
	//значение красной состовляющей
	putText(joinImg, std::to_string(TraffMem[TraffMem.size()-1].E1C), Point(200, 50),  FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	putText(joinImg, std::to_string(TraffMem[TraffMem.size()-1].E2C), Point(200, 100), FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	putText(joinImg, std::to_string(TraffMem[TraffMem.size()-1].E3C), Point(200, 150), FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	putText(joinImg, std::to_string(TraffMem[TraffMem.size()-1].E4C), Point(200, 200), FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	putText(joinImg, std::to_string(TraffMem[TraffMem.size()-1].E5C), Point(200, 250), FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	//значение размера овала максимально большой зоны
	putText(joinImg, std::to_string(TraffMem[TraffMem.size()-1].E1V), Point(200, 40),  FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	putText(joinImg, std::to_string(TraffMem[TraffMem.size()-1].E2V), Point(200, 90),  FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	putText(joinImg, std::to_string(TraffMem[TraffMem.size()-1].E3V), Point(200, 140), FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	putText(joinImg, std::to_string(TraffMem[TraffMem.size()-1].E4V), Point(200, 190), FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	putText(joinImg, std::to_string(TraffMem[TraffMem.size()-1].E5V), Point(200, 240), FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	//значение белой состовляющей зоны интереса
	putText(joinImg, std::to_string(TraffMem[TraffMem.size()-1].E1B), Point(200, 30),  FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	putText(joinImg, std::to_string(TraffMem[TraffMem.size()-1].E2B), Point(200, 80),  FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	putText(joinImg, std::to_string(TraffMem[TraffMem.size()-1].E3B), Point(200, 130), FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	putText(joinImg, std::to_string(TraffMem[TraffMem.size()-1].E4B), Point(200, 180), FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	putText(joinImg, std::to_string(TraffMem[TraffMem.size()-1].E5B), Point(200, 230), FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	//среднее по средним цветовой состовляющей
	putText(joinImg, std::to_string((int)E1CAverage[E1CAverage.size() - 1].Average2), Point(200, 20), FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	putText(joinImg, std::to_string((int)E2CAverage[E2CAverage.size() - 1].Average2), Point(200, 70), FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	putText(joinImg, std::to_string((int)E3CAverage[E3CAverage.size() - 1].Average2), Point(200, 120), FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	putText(joinImg, std::to_string((int)E4CAverage[E4CAverage.size() - 1].Average2), Point(200, 170), FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	putText(joinImg, std::to_string((int)E5CAverage[E5CAverage.size() - 1].Average2), Point(200, 220), FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));

	putText(joinImg, std::to_string(TraffMem[TraffMem.size()-1].RoiGrayGain), Point(200, 280), FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	putText(joinImg, std::to_string(TraffMem[TraffMem.size()-1].UseErosion), Point(200, 270), FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));
	putText(joinImg, std::to_string(MaxColorAverage), Point(0, 280), FONT_HERSHEY_COMPLEX, 0.4, Scalar::all(255));

	if (TraffMem[TraffMem.size() - 1].E1 == true) circle(joinImg, Point(250, 25), 10, TraffMem[TraffMem.size() - 1].E1ScalarColor, 20, 9);
	if (TraffMem[TraffMem.size() - 1].E2 == true) circle(joinImg, Point(250, 75), 10, TraffMem[TraffMem.size() - 1].E2ScalarColor, 20, 9);
	if (TraffMem[TraffMem.size() - 1].E3 == true) circle(joinImg, Point(250, 125), 10, TraffMem[TraffMem.size() - 1].E3ScalarColor, 20, 9);
	if (TraffMem[TraffMem.size() - 1].E4 == true) circle(joinImg, Point(250, 175), 10, TraffMem[TraffMem.size() - 1].E4ScalarColor, 20, 9);
	if (TraffMem[TraffMem.size() - 1].E5 == true) circle(joinImg, Point(250, 225), 10, TraffMem[TraffMem.size() - 1].E5ScalarColor, 20, 9);

	//разметка в ROI
	line(joinImg, Point(0, 50), Point(pict_width, 50), Scalar(255, 255, 0), 0, 0);
	line(joinImg, Point(0, 100), Point(pict_width, 100), Scalar(255, 255, 0), 0, 0);
	line(joinImg, Point(0, 150), Point(pict_width, 150), Scalar(255, 255, 0), 0, 0);
	line(joinImg, Point(0, 200), Point(pict_width, 200), Scalar(255, 255, 0), 0, 0);
	line(joinImg, Point(0, 250), Point(pict_width, 250), Scalar(255, 255, 0), 0, 0);

	rectangle(joinImg, Point(150, 255), Point(190, 285), Scalar::all(255)); //зона вывода бинарных данных

	//Отрисовываем историю и считаем статистику
	if (TraffMem.size() > 2)
	{			
		for (int i = 0; i < TraffMem.size(); i++)
		{
			//график цветной состовляющей
			if (ShowGraph == 0)
			{
				if (TraffMem[i].E1C>500) E1C = 29;
				else E1C = TraffMem[i].E1C/100;

				if (TraffMem[i].E2C>500) E2C = 29;
				else E2C = TraffMem[i].E2C/100;

				if (TraffMem[i].E3C>500) E3C = 29;
				else E3C = TraffMem[i].E3C/100;

				if (TraffMem[i].E4C>500) E4C = 29;
				else E4C = TraffMem[i].E4C/100;

				if (TraffMem[i].E5C>500) E5C = 29;
				else E5C = TraffMem[i].E5C/100;


				if (TraffMem[i + 1].E1C>500) E1C_1 = 29;
				else E1C_1 = TraffMem[i + 1].E1C/100;

				if (TraffMem[i + 1].E2C>500) E2C_1 = 29;
				else E2C_1 = TraffMem[i + 1].E2C/100;

				if (TraffMem[i + 1].E3C>500) E3C_1 = 29;
				else E3C_1 = TraffMem[i + 1].E3C/100;

				if (TraffMem[i + 1].E4C>500) E4C_1 = 29;
				else E4C_1 = TraffMem[i + 1].E4C/100;

				if (TraffMem[i + 1].E5C>500) E5C_1 = 29;
				else E5C_1 = TraffMem[i + 1].E5C/100;
			}
			//график колебания центра зоны интереса
			if (ShowGraph == 1)
			{
				if (TraffMem[i].E1V_CENTER.y > 0) E1C = ((TraffMem[i].E1V_CENTER.y + (TraffMem[i].E1V_CENTER.x - 150)) / 4) + 50;
				else E1C = 0;

				if (TraffMem[i].E2V_CENTER.y > 0) E2C = (((TraffMem[i].E2V_CENTER.y - 50) + (TraffMem[i].E2V_CENTER.x - 150)) / 4) + 50;
				else E2C = 0;

				if (TraffMem[i].E3V_CENTER.y > 0) E3C = (((TraffMem[i].E3V_CENTER.y - 100) + (TraffMem[i].E3V_CENTER.x - 150)) / 4) + 50;
				else E3C = 0;

				if (TraffMem[i].E4V_CENTER.y > 0) E4C = (((TraffMem[i].E4V_CENTER.y - 150) + (TraffMem[i].E4V_CENTER.x - 150)) / 4) + 50;
				else E4C = 0;

				if (TraffMem[i].E5V_CENTER.y > 0) E5C = (((TraffMem[i].E5V_CENTER.y - 200) + (TraffMem[i].E5V_CENTER.x - 150)) / 4) + 50;
				else E5C = 0;


				if (TraffMem[i + 1].E1V_CENTER.y > 0) E1C_1 = ((TraffMem[i + 1].E1V_CENTER.y + (TraffMem[i + 1].E1V_CENTER.x - 150)) / 4) + 50;
				else E1C_1 = 0;

				if (TraffMem[i + 1].E2V_CENTER.y > 0) E2C_1 = (((TraffMem[i + 1].E2V_CENTER.y - 50) + (TraffMem[i + 1].E2V_CENTER.x - 150)) / 4) + 50;
				else E2C_1 = 0;

				if (TraffMem[i + 1].E3V_CENTER.y > 0) E3C_1 = (((TraffMem[i + 1].E3V_CENTER.y - 100) + (TraffMem[i + 1].E3V_CENTER.x - 150)) / 4) + 50;
				else E3C_1 = 0;

				if (TraffMem[i + 1].E4V_CENTER.y > 0) E4C_1 = (((TraffMem[i + 1].E4V_CENTER.y - 150) + (TraffMem[i + 1].E4V_CENTER.x - 150)) / 4) + 50;
				else E4C_1 = 0;

				if (TraffMem[i + 1].E5V_CENTER.y > 0) E5C_1 = (((TraffMem[i + 1].E5V_CENTER.y - 200) + (TraffMem[i + 1].E5V_CENTER.x - 150)) / 4) + 50;
				else E5C_1 = 0;

			}
			
			if (ShowGraph == 2)
			{
				if (TraffMem[i].E1 == true) E1C = 25; else E1C = 0;
				if (TraffMem[i].E2 == true) E2C = 25; else E2C = 0;
				if (TraffMem[i].E3 == true) E3C = 25; else E3C = 0;	
				if (TraffMem[i].E4 == true) E4C = 25; else E4C = 0;
				if (TraffMem[i].E5 == true) E5C = 25; else E5C = 0;
				
				if (TraffMem[i + 1].E1 == true) E1C_1 = 25; else E1C_1 = 0;
				if (TraffMem[i + 1].E2 == true) E2C_1 = 25; else E2C_1 = 0;
				if (TraffMem[i + 1].E3 == true) E3C_1 = 25; else E3C_1 = 0;
				if (TraffMem[i + 1].E4 == true) E4C_1 = 25; else E4C_1 = 0;
				if (TraffMem[i + 1].E5 == true) E5C_1 = 25; else E5C_1 = 0;
			}
			if (ShowGraph == 3)
			{
				
				if (TraffMem[i].E1 == true) E1C = 23;  else E1C = 0;
				if (TraffMem[i].E2 == true) E2C = 23;  else E2C = 0;
				if (TraffMem[i].E3 == true) E3C = 23;  else E3C = 0; 
				if (TraffMem[i].E4 == true) E4C = 23;  else E4C = 0;
				if (TraffMem[i].E5 == true) E5C = 23;  else E5C = 0;
				
				if (TraffMem[i + 1].E1 == true) E1C_1 = 23; else E1C_1 = 0;	
				if (TraffMem[i + 1].E2 == true) E2C_1 = 23; else E2C_1 = 0;
				if (TraffMem[i + 1].E3 == true) E3C_1 = 23; else E3C_1 = 0;	
				if (TraffMem[i + 1].E4 == true) E4C_1 = 23; else E4C_1 = 0;	
				if (TraffMem[i + 1].E5 == true) E5C_1 = 23;	else E5C_1 = 0;	   
				
				if (TraffMem[i].E1Tcl == true) E1Tcl = 23;  else E1Tcl = 0;
				if (TraffMem[i].E2Tcl == true) E2Tcl = 23;  else E2Tcl = 0;
				if (TraffMem[i].E3Tcl == true) E3Tcl = 23;  else E3Tcl = 0;
				if (TraffMem[i].E4Tcl == true) E4Tcl = 23;  else E4Tcl = 0;
				if (TraffMem[i].E5Tcl == true) E5Tcl = 23;  else E5Tcl = 0;
				
				if (TraffMem[i + 1].E1Tcl == true) E1Tcl_1 = 23; else E1Tcl_1 = 0;
				if (TraffMem[i + 1].E2Tcl == true) E2Tcl_1 = 23; else E2Tcl_1 = 0;
				if (TraffMem[i + 1].E3Tcl == true) E3Tcl_1 = 23; else E3Tcl_1 = 0;
				if (TraffMem[i + 1].E4Tcl == true) E4Tcl_1 = 23; else E4Tcl_1 = 0;
				if (TraffMem[i + 1].E5Tcl == true) E5Tcl_1 = 23; else E5Tcl_1 = 0; 					

				line(joinImg, Point(i + 270, E1Tcl_1), Point(i + 270, E1Tcl), Scalar(10, 255, 203), 0, 0);
				line(joinImg, Point(i + 270, 51 + E2Tcl_1), Point(i + 270, 51 + E2Tcl), Scalar(10, 255, 203), 0, 0);
				line(joinImg, Point(i + 270, 101 + E3Tcl_1), Point(i + 270, 101 + E3Tcl), Scalar(10, 255, 203), 0, 0);
				line(joinImg, Point(i + 270, 151 + E4Tcl_1), Point(i + 270, 151 + E4Tcl), Scalar(10, 255, 203), 0, 0);
				line(joinImg, Point(i + 270, 201 + E5Tcl_1), Point(i + 270, 201 + E5Tcl), Scalar(10, 255, 203), 0, 0);	 
			}
			
			line(joinImg,
				Point(i + 270, 50 - E1C_1),
				Point(i + 270, 50 - E1C),
				Scalar(255, 150, 20), 0, 0);
			line(joinImg,
				Point(i + 270, 100 - E2C_1),
				Point(i + 270, 100 - E2C),
				Scalar(255, 150, 20), 0, 0);
			line(joinImg,
				Point(i + 270, 150 - E3C_1),
				Point(i + 270, 150 - E3C),
				Scalar(255, 150, 20), 0, 0);
			line(joinImg,
				Point(i + 270, 200 - E4C_1),
				Point(i + 270, 200 - E4C),
				Scalar(255, 150, 20), 0, 0);
			line(joinImg,
				Point(i + 270, 250 - E5C_1),
				Point(i + 270, 250 - E5C),
				Scalar(255, 150, 20), 0, 0);
			line(joinImg,
				Point(i + 270, 288 - (TraffMem[i + 1].RoiGrayGain)),
				Point(i + 270, 288 - (TraffMem[i].RoiGrayGain)),
				Scalar(255, 150, 20), 0, 0);
			line(joinImg,
				Point(i + 270, 252 + (TraffMem[i + 1].UseErosion * 2)),
				Point(i + 270, 252 + (TraffMem[i].UseErosion * 2)),
				Scalar(255, 0, 255), 0, 0);	
		}
		if (ShowGraph == 3)
		{
			/*Сравнительный Анализ работы детектора по каналу 1  */
			if ((TraffMem[TraffMem.size()].E1Tcl == true) &&	(TraffMem[TraffMem.size() - 1].E1Tcl == true) &&	(E1TLCState == 0))	E1TLCState = 1;			// TCL first ON 		
			if ((TraffMem[TraffMem.size()].E1 == true) &&		(TraffMem[TraffMem.size() - 1].E1 == true) &&		(E1TLCState == 1))	E1TLCState = 2;			// TCL ON |DET first ON
			else if ((TraffMem[TraffMem.size()].E1 == true) &&	(TraffMem[TraffMem.size() - 1].E1 == true) &&		(E1TLCState == 2))	E1TLCState = 3;			// TCL ON |DET next ON 		
			if ((TraffMem[TraffMem.size()].E1Tcl == false) &&   (TraffMem[TraffMem.size() - 1].E1Tcl == false) &&	(E1TLCState == 3))	E1TLCState = 4;			// TCL OFF
			if ((TraffMem[TraffMem.size()].E1 == false) &&		(TraffMem[TraffMem.size() - 1].E1 == false) &&		(E1TLCState == 4))	E1TLCState = 5;			// TCL OFF |DET first OFF 	
			else if ((TraffMem[TraffMem.size()].E1 == false) &&	(TraffMem[TraffMem.size() - 1].E1 == false) &&	    (E1TLCState == 5))						    // TCL OFF |DET next OFF
			{
				E1TLCState = 0;
				E1TCLPhaseCount++;
			}			
			
			
			if		((E1TLCState == 0) && (TraffMem[TraffMem.size() - 1].E1 == true))	{E1COutLag++;} 			
			if		(E1TLCState == 0)													{E1CStartLag_tmp = 0; E1CEndLag_tmp = 0;} 
			else if (E1TLCState == 1)													{E1CStartLag_tmp++;}
			else if (E1TLCState == 2)													{E1CStartLag = E1CStartLag_tmp;}
			else if ((E1TLCState == 3) && (TraffMem[TraffMem.size() - 1].E1 == false))	{E1CInLag++;}
			else if (E1TLCState == 4)													{E1CEndLag_tmp++;}
			else if (E1TLCState == 5)													{E1CEndLag = E1CEndLag_tmp;}
			putText(joinImg, "E1 State=" + to_string(E1TLCState) + 
							 "|SL=" + to_string(E1CStartLag) + 
							 "|EL=" + to_string(E1CEndLag) +
							 "|IL=" + to_string(E1CInLag) +	
							 "|OL=" + to_string(E1COutLag) +
							 "|PC=" + to_string(E1TCLPhaseCount), Point(0, 300), 0, 0.4, Scalar::all(255));
			
			/*Сравнительный Анализ работы детектора по каналу 2  */
			if ((TraffMem[TraffMem.size()].E2Tcl == true) &&	(TraffMem[TraffMem.size() - 1].E2Tcl == true) &&	(E2TLCState == 0))	E2TLCState = 1;			// TCL first ON 		
			if ((TraffMem[TraffMem.size()].E2 == true) &&		(TraffMem[TraffMem.size() - 1].E2 == true) &&		(E2TLCState == 1))	E2TLCState = 2;			// TCL ON |DET first ON
			else if ((TraffMem[TraffMem.size()].E2 == true) &&	(TraffMem[TraffMem.size() - 1].E2 == true) &&		(E2TLCState == 2))	E2TLCState = 3;			// TCL ON |DET next ON 		
			if ((TraffMem[TraffMem.size()].E2Tcl == false) &&   (TraffMem[TraffMem.size() - 1].E2Tcl == false) &&	(E2TLCState == 3))	E2TLCState = 4;			// TCL OFF
			if ((TraffMem[TraffMem.size()].E2 == false) &&		(TraffMem[TraffMem.size() - 1].E2 == false) &&		(E2TLCState == 4))	E2TLCState = 5;			// TCL OFF |DET first OFF 	
			else if ((TraffMem[TraffMem.size()].E2 == false) &&	(TraffMem[TraffMem.size() - 1].E2 == false) &&	    (E2TLCState == 5))							// TCL OFF |DET next OFF
			{
				E2TLCState = 0;
				E2TCLPhaseCount++;
			}
			
			if ((E2TLCState == 0) && (TraffMem[TraffMem.size() - 1].E2 == true))	{E2COutLag++; } 			
			if (E2TLCState == 0)													{E2CStartLag_tmp = 0; E2CEndLag_tmp = 0; } 
			else if (E2TLCState == 1)													{E2CStartLag_tmp++; }
			else if (E2TLCState == 2)													{E2CStartLag = E2CStartLag_tmp; }
			else if ((E2TLCState == 3) && (TraffMem[TraffMem.size() - 1].E2 == false))	{E2CInLag++; }
			else if (E2TLCState == 4)													{E2CEndLag_tmp++; }
			else if (E2TLCState == 5)													{E2CEndLag = E2CEndLag_tmp; }
			putText(joinImg,
				"E2 State=" + to_string(E2TLCState) + 
							 "|SL=" + to_string(E2CStartLag) + 
							 "|EL=" + to_string(E2CEndLag) +
							 "|IL=" + to_string(E2CInLag) +	
							 "|OL=" + to_string(E2COutLag) +
							 "|PC=" + to_string(E2TCLPhaseCount),
				Point(0, 310),
				0,
				0.4,
				Scalar::all(255));
			
			/*Сравнительный Анализ работы детектора по каналу 3  */
			if ((TraffMem[TraffMem.size()].E3Tcl == true) &&	(TraffMem[TraffMem.size() - 1].E3Tcl == true) &&	(E3TLCState == 0))	E3TLCState = 1;			// TCL first ON 		
			if ((TraffMem[TraffMem.size()].E3 == true) &&		(TraffMem[TraffMem.size() - 1].E3 == true) &&		(E3TLCState == 1))	E3TLCState = 2;			// TCL ON |DET first ON
			else if ((TraffMem[TraffMem.size()].E3 == true) &&	(TraffMem[TraffMem.size() - 1].E3 == true) &&		(E3TLCState == 2))	E3TLCState = 3;			// TCL ON |DET next ON 		
			if ((TraffMem[TraffMem.size()].E3Tcl == false) &&   (TraffMem[TraffMem.size() - 1].E3Tcl == false) &&	(E3TLCState == 3))	E3TLCState = 4;			// TCL OFF
			if ((TraffMem[TraffMem.size()].E3 == false) &&		(TraffMem[TraffMem.size() - 1].E3 == false) &&		(E3TLCState == 4))	E3TLCState = 5;			// TCL OFF |DET first OFF 	
			else if ((TraffMem[TraffMem.size()].E3 == false) &&	(TraffMem[TraffMem.size() - 1].E3 == false) &&	    (E3TLCState == 5))							// TCL OFF |DET next OFF
			{
				E3TLCState = 0;
				E3TCLPhaseCount++;
			}
			
			if ((E3TLCState == 0) && (TraffMem[TraffMem.size() - 1].E3 == true))	{E3COutLag++; } 			
			if (E3TLCState == 0)													{E3CStartLag_tmp = 0; E3CEndLag_tmp = 0; } 
			else if (E3TLCState == 1)													{E3CStartLag_tmp++; }
			else if (E3TLCState == 2)													{E3CStartLag = E3CStartLag_tmp; }
			else if ((E3TLCState == 3) && (TraffMem[TraffMem.size() - 1].E3 == false))	{E3CInLag++; }
			else if (E3TLCState == 4)													{E3CEndLag_tmp++; }
			else if (E3TLCState == 5)													{E3CEndLag = E3CEndLag_tmp; }
			putText(joinImg,
				"E3 State=" + to_string(E3TLCState) + 
							 "|SL=" + to_string(E3CStartLag) + 
							 "|EL=" + to_string(E3CEndLag) +
							 "|IL=" + to_string(E3CInLag) +	
							 "|OL=" + to_string(E3COutLag) +
							 "|PC=" + to_string(E3TCLPhaseCount),
				Point(0, 320),
				0,
				0.4,
				Scalar::all(255));
			
			/*Сравнительный Анализ работы детектора по каналу 4  */
			if ((TraffMem[TraffMem.size()].E4Tcl == true) &&	(TraffMem[TraffMem.size() - 1].E4Tcl == true) &&	(E4TLCState == 0))	E4TLCState = 1;			// TCL first ON 		
			if ((TraffMem[TraffMem.size()].E4 == true) &&		(TraffMem[TraffMem.size() - 1].E4 == true) &&		(E4TLCState == 1))	E4TLCState = 2;			// TCL ON |DET first ON
			else if ((TraffMem[TraffMem.size()].E4 == true) &&	(TraffMem[TraffMem.size() - 1].E4 == true) &&		(E4TLCState == 2))	E4TLCState = 3;			// TCL ON |DET next ON 		
			if ((TraffMem[TraffMem.size()].E4Tcl == false) &&   (TraffMem[TraffMem.size() - 1].E4Tcl == false) &&	(E4TLCState == 3))	E4TLCState = 4;			// TCL OFF
			if ((TraffMem[TraffMem.size()].E4 == false) &&		(TraffMem[TraffMem.size() - 1].E4 == false) &&		(E4TLCState == 4))	E4TLCState = 5;			// TCL OFF |DET first OFF 	
			else if ((TraffMem[TraffMem.size()].E4 == false) &&	(TraffMem[TraffMem.size() - 1].E4 == false) &&	    (E4TLCState == 5))							// TCL OFF |DET next OFF
			{
				E4TLCState = 0;
				E4TCLPhaseCount++;
			}
			
			if ((E4TLCState == 0) && (TraffMem[TraffMem.size() - 1].E4 == true))	{E4COutLag++; } 			
			if (E4TLCState == 0)													{E4CStartLag_tmp = 0; E4CEndLag_tmp = 0; } 
			else if (E4TLCState == 1)													{E4CStartLag_tmp++; }
			else if (E4TLCState == 2)													{E4CStartLag = E4CStartLag_tmp; }
			else if ((E4TLCState == 3) && (TraffMem[TraffMem.size() - 1].E4 == false))	{E4CInLag++; }
			else if (E4TLCState == 4)													{E4CEndLag_tmp++; }
			else if (E4TLCState == 5)													{E4CEndLag = E4CEndLag_tmp; }
			putText(joinImg,
				"E4 State=" + to_string(E4TLCState) + 
							 "|SL=" + to_string(E4CStartLag) + 
							 "|EL=" + to_string(E4CEndLag) +
							 "|IL=" + to_string(E4CInLag) +	
							 "|OL=" + to_string(E4COutLag) +
							 "|PC=" + to_string(E4TCLPhaseCount),
				Point(0, 330),
				0,
				0.4,
				Scalar::all(255));
			
			/*Сравнительный Анализ работы детектора по каналу 5  */
			if ((TraffMem[TraffMem.size()].E5Tcl == true) &&	(TraffMem[TraffMem.size() - 1].E5Tcl == true) &&	(E5TLCState == 0))	E5TLCState = 1;			// TCL first ON 		
			if ((TraffMem[TraffMem.size()].E5 == true) &&		(TraffMem[TraffMem.size() - 1].E5 == true) &&		(E5TLCState == 1))	E5TLCState = 2;			// TCL ON |DET first ON
			else if ((TraffMem[TraffMem.size()].E5 == true) &&	(TraffMem[TraffMem.size() - 1].E5 == true) &&		(E5TLCState == 2))	E5TLCState = 3;			// TCL ON |DET next ON 		
			if ((TraffMem[TraffMem.size()].E5Tcl == false) &&   (TraffMem[TraffMem.size() - 1].E5Tcl == false) &&	(E5TLCState == 3))	E5TLCState = 4;			// TCL OFF
			if ((TraffMem[TraffMem.size()].E5 == false) &&		(TraffMem[TraffMem.size() - 1].E5 == false) &&		(E5TLCState == 4))	E5TLCState = 5;			// TCL OFF |DET first OFF 	
			else if ((TraffMem[TraffMem.size()].E5 == false) &&	(TraffMem[TraffMem.size() - 1].E5 == false) &&	    (E5TLCState == 5))							// TCL OFF |DET next OFF
			{
				E5TLCState = 0;
				E5TCLPhaseCount++;
			}
			
			if ((E5TLCState == 0) && (TraffMem[TraffMem.size() - 1].E5 == true))	{E5COutLag++; } 			
			if (E5TLCState == 0)													{E5CStartLag_tmp = 0; E5CEndLag_tmp = 0; } 
			else if (E5TLCState == 1)													{E5CStartLag_tmp++; }
			else if (E5TLCState == 2)													{E5CStartLag = E5CStartLag_tmp; }
			else if ((E5TLCState == 3) && (TraffMem[TraffMem.size() - 1].E5 == false))	{E5CInLag++; }
			else if (E5TLCState == 4)													{E5CEndLag_tmp++; }
			else if (E5TLCState == 5)													{E5CEndLag = E5CEndLag_tmp; }
			putText(joinImg,
				"E5 State=" + to_string(E5TLCState) + 
							 "|SL=" + to_string(E5CStartLag) + 
							 "|EL=" + to_string(E5CEndLag) +
							 "|IL=" + to_string(E5CInLag) +	
							 "|OL=" + to_string(E5COutLag) +
							 "|PC=" + to_string(E4TCLPhaseCount),
				Point(0, 340),
				0,
				0.4,
				Scalar::all(255));
		}		
	}

	tm localTime;
	std::chrono::system_clock::time_point t = std::chrono::system_clock::now();
	time_t now = std::chrono::system_clock::to_time_t(t);
	localtime_r(&now, &localTime);
	const std::chrono::duration<double> tse = t.time_since_epoch();
	std::chrono::seconds::rep milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(tse).count() % 1000;	

	string Time = "Time:" + to_string(1900 + localTime.tm_year) + '.' + to_string(localTime.tm_mon + 1) + '.' + to_string(localTime.tm_mday) + ' '
		+ to_string(localTime.tm_hour) + ':' + to_string(localTime.tm_min) + ':' + to_string(localTime.tm_sec) + '.' + to_string(milliseconds);
	putText(joinImg, Time, Point(0, 260), FONT_HERSHEY_SIMPLEX, 0.3, Scalar::all(255));	   	

	if (TraffMem[TraffMem.size() - 1].E1 == true) ellipse(joinImg, Point(160, 260), Size(1, 1), 1, 1, 1, Scalar::all(255), 1, 1); 
	if (TraffMem[TraffMem.size() - 1].E2 == true) ellipse(joinImg, Point(161, 260), Size(1, 1), 1, 1, 1, Scalar::all(255), 1, 1);
	if (TraffMem[TraffMem.size() - 1].E3 == true) ellipse(joinImg, Point(162, 260), Size(1, 1), 1, 1, 1, Scalar::all(255), 1, 1);
	if (TraffMem[TraffMem.size() - 1].E4 == true) ellipse(joinImg, Point(163, 260), Size(1, 1), 1, 1, 1, Scalar::all(255), 1, 1);
	if (TraffMem[TraffMem.size() - 1].E5 == true) ellipse(joinImg, Point(164, 260), Size(1, 1), 1, 1, 1, Scalar::all(255), 1, 1);

	if (TraffMem[TraffMem.size() - 1].E1Tcl == true) ellipse(joinImg, Point(160, 262), Size(1, 1), 1, 1, 1, Scalar::all(255), 1, 1);
	if (TraffMem[TraffMem.size() - 1].E2Tcl == true) ellipse(joinImg, Point(161, 262), Size(1, 1), 1, 1, 1, Scalar::all(255), 1, 1);
	if (TraffMem[TraffMem.size() - 1].E3Tcl == true) ellipse(joinImg, Point(162, 262), Size(1, 1), 1, 1, 1, Scalar::all(255), 1, 1);
	if (TraffMem[TraffMem.size() - 1].E4Tcl == true) ellipse(joinImg, Point(163, 262), Size(1, 1), 1, 1, 1, Scalar::all(255), 1, 1);
	if (TraffMem[TraffMem.size() - 1].E5Tcl == true) ellipse(joinImg, Point(160, 260), Size(1, 1), 1, 1, 1, Scalar::all(255), 1, 1);
	
	if (E1CStartLag > 150){	E1CStartLag = 0; }
	if (E2CStartLag > 150){	E2CStartLag = 0; }
	if (E3CStartLag > 150){	E3CStartLag = 0; }
	if (E4CStartLag > 150){	E4CStartLag = 0; }
	if (E5CStartLag > 150){	E5CStartLag = 0; }
	
	if (E1CEndLag > 150){E1CEndLag = 0; }
	if (E2CEndLag > 150){E2CEndLag = 0; }
	if (E3CEndLag > 150){E3CEndLag = 0; }
	if (E4CEndLag > 150){E4CEndLag = 0; }
	if (E5CEndLag > 150){E5CEndLag = 0; }
	
	
}

void findContours()
{
	E1V = E2V = E3V = E4V = E5V = 0;
	E1V_CENTER = E2V_CENTER = E3V_CENTER = E4V_CENTER = E5V_CENTER = Point(0, 0);
	Scalar color = Scalar(255, 255, 0);
	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	blur(RoiGray, RoiGray, Size(3, 3));
	threshold(RoiGray, threshold_output, thresh, 255, THRESH_BINARY);
	findContours(threshold_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
	vector<RotatedRect> minRect(contours.size());
	vector<RotatedRect> minEllipse(contours.size());

	for (size_t i = 0; i < contours.size(); i++)
	{
		minRect[i] = minAreaRect(Mat(contours[i]));
		if (contours[i].size() > 5)
		{
			minEllipse[i] = fitEllipse(Mat(contours[i]));
		}
	}
	RoiContours = Mat::zeros(threshold_output.size(), CV_8UC3);
	for (size_t i = 0; i< contours.size(); i++)
	{
		RotatedRect rRect = minEllipse[i];
		if ((rRect.center.y >(0 + DeadZone)) && (rRect.center.y < (50 - DeadZone)) && (rRect.center.x > DeadZone) && (rRect.center.x < (50 - DeadZone)))
		{
			if (E1V < (rRect.size.height + rRect.size.width))
			{
				E1V = rRect.size.height + rRect.size.width;
				E1V_CENTER = rRect.center;
				ellipse(RoiContours, rRect, color, 2, 8);
				ellipse(RoiContours, rRect.center, cv::Size(1, 1), 1, 1, 1, color, 2, 8);
			}
		}
		if ((rRect.center.y >(50 + DeadZone)) && (rRect.center.y < (100 - DeadZone)) && (rRect.center.x > DeadZone) && (rRect.center.x < (50 - DeadZone)))
		{
			if (E2V < (rRect.size.height + rRect.size.width))
			{
				E2V = rRect.size.height + rRect.size.width;
				E2V_CENTER = rRect.center;
				ellipse(RoiContours, rRect, color, 2, 8);
				ellipse(RoiContours, rRect.center, cv::Size(1, 1), 1, 1, 1, color, 2, 8);
			}
		}
		if ((rRect.center.y >(100 + DeadZone)) && (rRect.center.y < (150 - DeadZone)) && (rRect.center.x > DeadZone) && (rRect.center.x < (50 - DeadZone)))
		{
			if (E3V < (rRect.size.height + rRect.size.width))
			{
				E3V = rRect.size.height + rRect.size.width;
				E3V_CENTER = rRect.center;
				ellipse(RoiContours, rRect, color, 2, 8);
				ellipse(RoiContours, rRect.center, cv::Size(1, 1), 1, 1, 1, color, 2, 8);
			}
		}
		if ((rRect.center.y >(150 + DeadZone)) && (rRect.center.y < (200 - DeadZone)) && (rRect.center.x > DeadZone) && (rRect.center.x < (50 - DeadZone)))
		{
			if (E4V < (rRect.size.height + rRect.size.width))
			{
				E4V = rRect.size.height + rRect.size.width;
				E4V_CENTER = rRect.center;
				ellipse(RoiContours, rRect, color, 2, 8);
				ellipse(RoiContours, rRect.center, cv::Size(1, 1), 1, 1, 1, color, 2, 8);
			}
		}
		if ((rRect.center.y >(200 + DeadZone)) && (rRect.center.y < (250 - DeadZone)) && (rRect.center.x > DeadZone) && (rRect.center.x < (50 - DeadZone)))
		{
			if (E5V < (rRect.size.height + rRect.size.width))
			{
				E5V = rRect.size.height + rRect.size.width;
				E5V_CENTER = rRect.center;
				ellipse(RoiContours, rRect, color, 2, 8);
				ellipse(RoiContours, rRect.center, cv::Size(1, 1), 1, 1, 1, color, 2, 8);
			}
		}
	}
}

void findContours(Mat GrayPicIn)
{
	E1V = E2V = E3V = E4V = E5V = 0;
	E1V_CENTER = E2V_CENTER = E3V_CENTER = E4V_CENTER = E5V_CENTER = Point(0, 0);
	Scalar color = Scalar(255, 255, 0);
	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	blur(GrayPicIn, GrayPicIn, Size(3, 3));
	threshold(GrayPicIn, threshold_output, thresh, 255, THRESH_BINARY);
	findContours(threshold_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
	vector<RotatedRect> minRect(contours.size());
	vector<RotatedRect> minEllipse(contours.size());

	for (size_t i = 0; i < contours.size(); i++)
	{
		minRect[i] = minAreaRect(Mat(contours[i]));
		if (contours[i].size() > 5)
		{
			minEllipse[i] = fitEllipse(Mat(contours[i]));
		}
	}
	RoiContours = Mat::zeros(threshold_output.size(), CV_8UC3);
	for (size_t i = 0; i< contours.size(); i++)
	{
		RotatedRect rRect = minEllipse[i];
		if ((rRect.center.y >(0 + DeadZone)) && (rRect.center.y < (50 - DeadZone)) && (rRect.center.x > DeadZone) && (rRect.center.x < (50 - DeadZone)))
		{
			if (E1V < (rRect.size.height + rRect.size.width))
			{
				E1V = rRect.size.height + rRect.size.width;
				E1V_CENTER = rRect.center;
				//ellipse(RoiContours, rRect, color, 2, 8);
				//ellipse(RoiContours, rRect.center, cv::Size(1, 1), 1, 1, 1, color, 2, 8);
			}
		}
		if ((rRect.center.y >(50 + DeadZone)) && (rRect.center.y < (100 - DeadZone)) && (rRect.center.x > DeadZone) && (rRect.center.x < (50 - DeadZone)))
		{
			if (E2V < (rRect.size.height + rRect.size.width))
			{
				E2V = rRect.size.height + rRect.size.width;
				E2V_CENTER = rRect.center;
				//ellipse(RoiContours, rRect, color, 2, 8);
				//ellipse(RoiContours, rRect.center, cv::Size(1, 1), 1, 1, 1, color, 2, 8);
			}
		}
		if ((rRect.center.y >(100 + DeadZone)) && (rRect.center.y < (150 - DeadZone)) && (rRect.center.x > DeadZone) && (rRect.center.x < (50 - DeadZone)))
		{
			if (E3V < (rRect.size.height + rRect.size.width))
			{
				E3V = rRect.size.height + rRect.size.width;
				E3V_CENTER = rRect.center;
				//ellipse(RoiContours, rRect, color, 2, 8);
				//ellipse(RoiContours, rRect.center, cv::Size(1, 1), 1, 1, 1, color, 2, 8);
			}
		}
		if ((rRect.center.y >(150 + DeadZone)) && (rRect.center.y < (200 - DeadZone)) && (rRect.center.x > DeadZone) && (rRect.center.x < (50 - DeadZone)))
		{
			if (E4V < (rRect.size.height + rRect.size.width))
			{
				E4V = rRect.size.height + rRect.size.width;
				E4V_CENTER = rRect.center;
				//ellipse(RoiContours, rRect, color, 2, 8);
				//ellipse(RoiContours, rRect.center, cv::Size(1, 1), 1, 1, 1, color, 2, 8);
			}
		}
		if ((rRect.center.y >(200 + DeadZone)) && (rRect.center.y < (250 - DeadZone)) && (rRect.center.x > DeadZone) && (rRect.center.x < (50 - DeadZone)))
		{
			if (E5V < (rRect.size.height + rRect.size.width))
			{
				E5V = rRect.size.height + rRect.size.width;
				E5V_CENTER = rRect.center;
				//ellipse(RoiContours, rRect, color, 2, 8);
				//ellipse(RoiContours, rRect.center, cv::Size(1, 1), 1, 1, 1, color, 2, 8);
			}
		}
	}
}	

TraffickLight Read_Config(TraffickLight TR)
{
	Scalar green = Scalar(0, 102, 0);
	Scalar yellow = Scalar(0, 179, 255);
	Scalar red = Scalar(0, 0, 230);
	
	INIReader reader("/etc/odyssey/Traff.ini");
	string channel_section = "c";
	
	/* вычитываем MAIN*/
	channelSharedMemGrabber =  (int)reader.GetInteger("main", "channelinputid", 0);
	gaussBlur =  (int)reader.GetInteger("main", "gaussBlur", 0);
	RoiGrayGain =  (int)reader.GetInteger("main", "RoiGrayGain", 0);
	DeadZone =  (int)reader.GetInteger("main", "DeadZone", 0);
	thresh =  (int)reader.GetInteger("main", "thresh", 0);
	UseErosion =  (int)reader.GetInteger("main", "UseErosion", 0);
	pict_width = (int)reader.GetInteger("main", "pict_width", 0);	
	ShowGraph =  (int)reader.GetInteger("main", "showgraph", 3);
	
	int RedInt =  (int)reader.GetInteger("main", "red", 3);
	int YellowInt =  (int)reader.GetInteger("main", "yellow", 3);
	int GreepInt =  (int)reader.GetInteger("main", "red", 3);
	
	
	/* заполняем квадраты всех облостей интереса*/
	for (int i = 1; i < 6; i++)
	{ 
		if ((int)reader.GetInteger(channel_section + to_string(i), "channeltype", 0) != 0)
		{
			ROI.push_back(Rect(
				(int)reader.GetInteger(channel_section + to_string(i), "x1", 0),
				(int)reader.GetInteger(channel_section + to_string(i), "y1", 0),
				(int)reader.GetInteger(channel_section + to_string(i), "width", 0),
				(int)reader.GetInteger(channel_section + to_string(i), "height", 0)));
		}
		else
		{
			ROI.push_back(Rect(0, 287, 1, 1));
		}		
	}
	
	/* заполняем настроки типа канала */
	TR.E1Color = (int)reader.GetInteger("c1", "channeltype", 0); 
	TR.E2Color = (int)reader.GetInteger("c2", "channeltype", 0); 
	TR.E3Color = (int)reader.GetInteger("c3", "channeltype", 0); 
	TR.E4Color = (int)reader.GetInteger("c4", "channeltype", 0); 
	TR.E5Color = (int)reader.GetInteger("c5", "channeltype", 0); 
	
	/* заполняем настройки соответствия каналов TCL*/
	TR.E1TclChannel = (int)reader.GetInteger("c1", "channeltcl", 0); 
	TR.E2TclChannel = (int)reader.GetInteger("c2", "channeltcl", 0);
	TR.E3TclChannel = (int)reader.GetInteger("c3", "channeltcl", 0);
	TR.E4TclChannel = (int)reader.GetInteger("c4", "channeltcl", 0);
	TR.E5TclChannel = (int)reader.GetInteger("c5", "channeltcl", 0);
	
	/* заполняем настройки цвета секции в Scalar и числовое представление пиксела*/
	if (TR.E1Color == 1)		{ TR.E1ScalarColor = red;		TR.E1PixelColor = RedInt;	} 		
	else if (TR.E1Color == 2)	{ TR.E1ScalarColor = yellow;	TR.E1PixelColor = YellowInt;}
	else if (TR.E1Color == 3) 	{ TR.E1ScalarColor = green;		TR.E1PixelColor = GreepInt;	}
	else if (TR.E1Color == 4)	{ TR.E1ScalarColor = red;		TR.E1PixelColor = GreepInt;	}
	
	if (TR.E2Color == 1) 		{TR.E2ScalarColor = red;		TR.E2PixelColor = RedInt;	}
	else if (TR.E2Color == 2)	{TR.E2ScalarColor = yellow;		TR.E2PixelColor = YellowInt;}
	else if (TR.E2Color == 3)	{TR.E2ScalarColor = green;		TR.E2PixelColor = GreepInt;	}
	else if (TR.E2Color == 4)	{TR.E2ScalarColor = red;		TR.E2PixelColor = GreepInt;	}
		
	if (TR.E3Color == 1)		{TR.E3ScalarColor = red;		TR.E3PixelColor = RedInt;	}
	else if (TR.E3Color == 2)  	{TR.E3ScalarColor = yellow;		TR.E3PixelColor = YellowInt;}
	else if (TR.E3Color == 3)	{TR.E3ScalarColor = green;		TR.E3PixelColor = GreepInt; }
	else if (TR.E3Color == 4)	{TR.E3ScalarColor = red;		TR.E3PixelColor = GreepInt; }
	
	if (TR.E4Color == 1) 	   	{TR.E4ScalarColor = red;		TR.E4PixelColor = RedInt;	}
	else if (TR.E4Color == 2)	{TR.E4ScalarColor = yellow;		TR.E4PixelColor = YellowInt;}
	else if (TR.E4Color == 3)	{TR.E4ScalarColor = green;		TR.E4PixelColor = GreepInt; }
	else if (TR.E4Color == 4)	{TR.E4ScalarColor = red;		TR.E4PixelColor = GreepInt; }
	
	if (TR.E5Color == 1) 		{TR.E5ScalarColor = red;		TR.E5PixelColor = RedInt;	}
	else if (TR.E5Color == 2)	{TR.E5ScalarColor = yellow;		TR.E5PixelColor = YellowInt;}
	else if (TR.E5Color == 3)	{TR.E5ScalarColor = green;		TR.E5PixelColor = GreepInt; }
	else if (TR.E5Color == 4)	{TR.E5ScalarColor = red;		TR.E5PixelColor = GreepInt; }
	
	return TR;
}

int main(int argc, char *argv[])
{ 	
	MaxColorAverage = 0;
	TraffickLight TrLi;
	TraffickColorAverage  ECAverage;
	memset(&TrLi, 0, sizeof(TraffickLight));
	memset(&ECAverage, 0, sizeof(TraffickColorAverage));
	E1CAverage.push_back(ECAverage);
	E2CAverage.push_back(ECAverage);
	E3CAverage.push_back(ECAverage);
	E4CAverage.push_back(ECAverage);
	E5CAverage.push_back(ECAverage);
	TrLi = Read_Config(TrLi);
	SharedMemGrabber grab(pict_width, pict_height, 10077 + channelSharedMemGrabber * 2);
	SharedMemGrabber outgrab(pict_width, pict_height + 96, 10077 + 23 * 2);
	//cout << "Start grab video" << endl;
	
	TclState state;
	imusoconn imuso;  
	
	int frame_n = 0;
	while (true)
	{
		if (!grab.getNextFrame())
		{ 
			usleep(1000); 
			continue; 
		}
		
		Mat image = Mat(pict_height, pict_width, CV_8UC2, grab.last_frame_videobuf);
		cvtColor(image, image, CV_YUV2BGR_YUY2);
		frame_n++;
		//cout << "Frame input " << frame_n << " image " << image.cols << "x" << image.rows << endl;

		//клеем все части картинок
		Color288x50 = Mat(1, 50, CV_8UC3) * 0;
		RoiHSV = Mat(1, 50, CV_8UC3) * 0;		
		Mat roiHSV = Mat(50, 50, CV_8UC3);
		empty = Mat(50, 50, CV_8UC3) * 0;
		vector<Mat> Layer;
		for (int i = 0; i < 5; i++)
		{
			if (ROI[i].width != 1)
			{	  				
				//клеем цветную картинку
				Mat roi = Mat(50, 50, CV_8UC3);
				resize(image(ROI[i]), roi, roi.size(), 0, 0, CV_INTER_LINEAR);
				vconcat(Color288x50, roi, Color288x50);

				//преобразуем и клеем HSV
				roi = Mat(image(ROI[i]).size(), CV_8UC3);
				cvtColor(image(ROI[i]), roi, CV_BGR2HSV);  //преобразуем цвет из BGR в HSV
				resize(roi, roiHSV, roiHSV.size(), 0, 0, CV_INTER_AREA);
				vconcat(RoiHSV, roiHSV, RoiHSV);
			}
			else
			{
				vconcat(Color288x50, empty, Color288x50);
				vconcat(RoiHSV, empty, RoiHSV);
			}
		}
		
		empty = Mat(37, 50, CV_8UC3) * 0;							    
		vconcat(Color288x50, empty, Color288x50);						//клеем пустое поле
		vconcat(RoiHSV, empty, RoiHSV);									//клеем пустое поле	
		TraffMem.push_back(TrLi);										//записываем предидущее состояние светофора	
		if (TraffMem.size() > 120) {TraffMem.erase(TraffMem.begin()); }	//устанавливаем длину истории работы светофора в 120 кадров 


		memset(&TrLi, 0, sizeof(TraffickLight));		
		/*Пересохраняем Тип цвета секции*/
		TrLi.E1Color = TraffMem[TraffMem.size() - 1].E1Color;
		TrLi.E2Color = TraffMem[TraffMem.size() - 1].E2Color;
		TrLi.E3Color = TraffMem[TraffMem.size() - 1].E3Color;
		TrLi.E4Color = TraffMem[TraffMem.size() - 1].E4Color;
		TrLi.E5Color = TraffMem[TraffMem.size() - 1].E5Color;
		
		/*Пересохраняем соответствие каналов TCL и детектора*/
		TrLi.E1TclChannel = TraffMem[TraffMem.size() - 1].E1TclChannel;
		TrLi.E2TclChannel = TraffMem[TraffMem.size() - 1].E2TclChannel;
		TrLi.E3TclChannel = TraffMem[TraffMem.size() - 1].E3TclChannel;
		TrLi.E4TclChannel = TraffMem[TraffMem.size() - 1].E4TclChannel;
		TrLi.E5TclChannel = TraffMem[TraffMem.size() - 1].E5TclChannel;
		
		/*Пересохраняем соответствие каналов TCL и детектора*/
		TrLi.E1ScalarColor = TraffMem[TraffMem.size() - 1].E1ScalarColor;
		TrLi.E2ScalarColor = TraffMem[TraffMem.size() - 1].E2ScalarColor;
		TrLi.E3ScalarColor = TraffMem[TraffMem.size() - 1].E3ScalarColor;
		TrLi.E4ScalarColor = TraffMem[TraffMem.size() - 1].E4ScalarColor;
		TrLi.E5ScalarColor = TraffMem[TraffMem.size() - 1].E5ScalarColor;
		
		/*Пересохраняем числовое представление значение детекции пиксела*/
		TrLi.E1PixelColor = TraffMem[TraffMem.size() - 1].E1PixelColor;
		TrLi.E2PixelColor = TraffMem[TraffMem.size() - 1].E2PixelColor;
		TrLi.E3PixelColor = TraffMem[TraffMem.size() - 1].E3PixelColor;
		TrLi.E4PixelColor = TraffMem[TraffMem.size() - 1].E4PixelColor;
		TrLi.E5PixelColor = TraffMem[TraffMem.size() - 1].E5PixelColor;
			
		uint8_t* pixelPtr = (uint8_t*)RoiHSV.data;
		int cn = RoiHSV.channels();
		for (int j = 0; j < RoiHSV.cols; j++)
		{
			for (int i = 0; i < 50; i++)    if (pixelPtr[i*RoiHSV.cols*cn + j*cn + 2] > TrLi.E1PixelColor) TrLi.E1C++;
			for (int i = 50; i < 100; i++)  if (pixelPtr[i*RoiHSV.cols*cn + j*cn + 2] > TrLi.E2PixelColor) TrLi.E2C++;
			for (int i = 100; i < 150; i++) if (pixelPtr[i*RoiHSV.cols*cn + j*cn + 2] > TrLi.E3PixelColor) TrLi.E3C++;
			for (int i = 150; i < 200; i++) if (pixelPtr[i*RoiHSV.cols*cn + j*cn + 2] > TrLi.E4PixelColor) TrLi.E4C++;
			for (int i = 200; i < 250; i++) if (pixelPtr[i*RoiHSV.cols*cn + j*cn + 2] > TrLi.E5PixelColor) TrLi.E5C++;
		}
		if (TrLi.E1C > 10)
		{
			ECAverage.Average2 = 0;
			ECAverage.Average = 0;
			ECAverage.C = TrLi.E1C;
			for (int i = 0; i < E1CAverage.size(); i++)
			{
				ECAverage.Average = ECAverage.Average + E1CAverage[i].C;
				ECAverage.Average2 = ECAverage.Average2 + E1CAverage[i].Average;
			}
			ECAverage.Average = ECAverage.Average / E1CAverage.size();
			ECAverage.Average2 = ECAverage.Average2 / E1CAverage.size();			
			E1CAverage.push_back(ECAverage);			
			if (E1CAverage.size() > 120) 
			{			
				E1CAverage.erase(E1CAverage.begin());
			}
		}
		if (TrLi.E2C > 10)
		{
			ECAverage.Average2 = 0;
			ECAverage.Average = 0;
			ECAverage.C = TrLi.E2C;
			for (int i = 0; i < E2CAverage.size(); i++)
			{
				ECAverage.Average = ECAverage.Average + E2CAverage[i].C;
				ECAverage.Average2 = ECAverage.Average2 + E2CAverage[i].Average;
			}
			ECAverage.Average = ECAverage.Average / E2CAverage.size();
			ECAverage.Average2 = ECAverage.Average2 / E2CAverage.size();			
			E2CAverage.push_back(ECAverage);			
			if (E2CAverage.size() > 120) 
			{			
				E2CAverage.erase(E2CAverage.begin());
			}
		}
		if (TrLi.E3C > 10)
		{
			ECAverage.Average2 = 0;
			ECAverage.Average = 0;
			ECAverage.C = TrLi.E3C;
			for (int i = 0; i < E3CAverage.size(); i++)
			{
				ECAverage.Average = ECAverage.Average + E3CAverage[i].C;
				ECAverage.Average2 = ECAverage.Average2 + E3CAverage[i].Average;
			}
			ECAverage.Average = ECAverage.Average / E3CAverage.size();
			ECAverage.Average2 = ECAverage.Average2 / E3CAverage.size();			
			E3CAverage.push_back(ECAverage);			
			if (E3CAverage.size() > 120) 
			{			
				E3CAverage.erase(E3CAverage.begin());
			}
		}
		if (TrLi.E4C > 10)
		{
			ECAverage.Average2 = 0;
			ECAverage.Average = 0;
			ECAverage.C = TrLi.E4C;
			for (int i = 0; i < E4CAverage.size(); i++)
			{
				ECAverage.Average = ECAverage.Average + E4CAverage[i].C;
				ECAverage.Average2 = ECAverage.Average2 + E4CAverage[i].Average;
			}
			ECAverage.Average = ECAverage.Average / E4CAverage.size();
			ECAverage.Average2 = ECAverage.Average2 / E4CAverage.size();			
			E4CAverage.push_back(ECAverage);			
			if (E4CAverage.size() > 120) 
			{			
				E4CAverage.erase(E4CAverage.begin());
			}
		}
		if (TrLi.E5C > 10)
		{
			ECAverage.Average2 = 0;
			ECAverage.Average = 0;
			ECAverage.C = TrLi.E4C;
			for (int i = 0; i < E5CAverage.size(); i++)
			{
				ECAverage.Average = ECAverage.Average + E5CAverage[i].C;
				ECAverage.Average2 = ECAverage.Average2 + E5CAverage[i].Average;
			}
			ECAverage.Average = ECAverage.Average / E5CAverage.size();
			ECAverage.Average2 = ECAverage.Average2 / E5CAverage.size();			
			E5CAverage.push_back(ECAverage);			
			if (E5CAverage.size() > 120) 
			{			
				E5CAverage.erase(E5CAverage.begin());
			}
		}
		
		
		/*находим максимальное из средних*/
		
		if ((E1CAverage.size() > 0) && (E2CAverage.size() > 0))
		{ 
			if (E1CAverage[E1CAverage.size() - 1].Average2 > E2CAverage[E2CAverage.size() - 1].Average2) 
			{MaxColorAverage = E1CAverage[E1CAverage.size() - 1].Average2; }
			else {	MaxColorAverage = E2CAverage[E2CAverage.size() - 1].Average2; }
		}
		if (E3CAverage.size() > 0)
		{
			if (E3CAverage[E3CAverage.size() - 1].Average2 > MaxColorAverage){ MaxColorAverage = E3CAverage[E3CAverage.size() - 1].Average2; }
		}
		if (E4CAverage.size())
		{
			if (E4CAverage[E4CAverage.size() - 1].Average2 > MaxColorAverage){ MaxColorAverage = E4CAverage[E4CAverage.size() - 1].Average2; }
		}
		if (E5CAverage.size())
		{
			if (E5CAverage[E5CAverage.size() - 1].Average2 > MaxColorAverage){ MaxColorAverage = E5CAverage[E5CAverage.size() - 1].Average2; }
		}
				
		GaussianBlur(RoiHSV, RoiHSV, Size(gaussBlur, gaussBlur), 0, 0);			
		split(RoiHSV, Layer);
		RoiGray = Layer[1] * 0.7;

		//если значение красной состовляющей больше 500 пикселей на зону то считаем это сработкой
		RoiGrayGain = 21;
		TrLi.UseErosion = 0;
		
		Mat RoiGrayTmp;
		if (TrLi.E1C > 500)	{ TrLi.E1 = 1; RoiGrayGain = 1; } else {	RoiGrayGain = 21; }						
		if (TrLi.E2C > 500) { TrLi.E2 = 1; RoiGrayGain = 1; } else {	RoiGrayGain = 21; }
		if (TrLi.E3C > 500) { TrLi.E3 = 1; RoiGrayGain = 1; } else {	RoiGrayGain = 21; }
		if (TrLi.E4C > 500) { TrLi.E4 = 1; RoiGrayGain = 1; } else {	RoiGrayGain = 21; }	  		
		if (TrLi.E5C > 500) { TrLi.E5 = 1; RoiGrayGain = 1; } else {	RoiGrayGain = 21; }
		
		//стабилизируем яркость картинки выбирая угловые зоны как самые не значимые	
		for (int b = 0; b < RoiGrayGain; b++)
		{
			TrLi.E1B = TrLi.E2B = TrLi.E3B = TrLi.E4B = TrLi.E5B = 0;
			TrLi.E1B_D = TrLi.E2B_D = TrLi.E3B_D = TrLi.E4B_D = TrLi.E5B_D = 0;
			TrLi.E1V_CENTER = TrLi.E2V_CENTER = TrLi.E3V_CENTER = TrLi.E4V_CENTER = TrLi.E5V_CENTER = Point(0, 0);
			RoiGrayTmp = RoiGray * (RoiGrayGain / 5);

			for (size_t i = 0; i < TrLi.UseErosion; i++)
			{
				int erosion_type;
				int erosion_size = 2;
				Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2 * erosion_size + 1, 2 * erosion_size + 1), Point(erosion_size, erosion_size));
				erode(RoiGrayTmp, RoiGrayTmp, element);
			} 			

			for (int j = 0; j < RoiGrayTmp.cols; j++)
			{
				for (int i = 0; i < 50; i++)    if (((i < DeadZone + 0) || (i > 50 - DeadZone)) && ((j < DeadZone) || (j  > 50 - DeadZone)))
				{
					ellipse(Color288x50, cv::Point(j, i), cv::Size(1, 1), 1, 1, 1, Scalar(255, 255, 0), 1, 1);
					if (RoiGrayTmp.at<uchar>(i, j) > 200) TrLi.E1B_D++;
				}
				else if (RoiGrayTmp.at<uchar>(i, j) > 200) TrLi.E1B++;

				for (int i = 50; i < 100; i++)  if (((i < DeadZone + 50) || (i > 100 - DeadZone)) && ((j < DeadZone) || (j  > 50 - DeadZone)))
				{
					if (RoiGrayTmp.at<uchar>(i, j) > 200) TrLi.E2B_D++;
				}
				else if (RoiGrayTmp.at<uchar>(i, j) > 200) TrLi.E2B++;

				for (int i = 100; i < 150; i++) if (((i < DeadZone + 100) || (i > 150 - DeadZone)) && ((j < DeadZone) || (j  > 50 - DeadZone)))
				{
					if (RoiGrayTmp.at<uchar>(i, j) > 200) TrLi.E3B_D++;
				}
				else if (RoiGrayTmp.at<uchar>(i, j) > 200) TrLi.E3B++;

				for (int i = 150; i < 200; i++) if (((i < DeadZone + 150) || (i > 200 - DeadZone)) && ((j < DeadZone) || (j  > 50 - DeadZone)))
				{
					if (RoiGrayTmp.at<uchar>(i, j) > 200) TrLi.E4B_D++;
				}
				else if (RoiGrayTmp.at<uchar>(i, j) > 200) TrLi.E4B++;

				for (int i = 200; i < 250; i++) if (((i < DeadZone + 200) || (i > 250 - DeadZone)) && ((j < DeadZone) || (j  > 50 - DeadZone)))
				{
					if (RoiGrayTmp.at<uchar>(i, j) > 200) TrLi.E5B_D++;
				}
				else if (RoiGrayTmp.at<uchar>(i, j) > 200) TrLi.E5B++;
			}	  
			if ((TrLi.E1B > 600) || (TrLi.E2B > 600) || (TrLi.E3B > 600) || (TrLi.E4B > 600) || (TrLi.E5B > 600))
			{
				TrLi.UseErosion++;
			}
			else
			{
				findContours(RoiGrayTmp);																					//ищем контуры ярких пятен
				if ((TrLi.E1B_D > 10) || (TrLi.E2B_D > 10) || (TrLi.E3B_D > 10) || (TrLi.E4B_D > 10) || (TrLi.E5B_D > 10))	//Проверяем количество ярких пикселов в мёртвой зоне
				{
					if ((TrLi.E1C < 1) || (TrLi.E2C < 1) || (TrLi.E3C < 1) || (TrLi.E4C < 1) || (TrLi.E5C < 1))				//проверяем количество красных пикселов
						RoiGrayGain--;																						//если их слишком много приглушаем яркость
				}
				else if ((E1V > 100) || (E2V > 100) || (E3V > 100) || (E4V > 100) || (E5V > 100))
				{
					RoiGrayGain--;
				}
				else
				{
					TrLi.RoiGrayGain = RoiGrayGain;
					b = RoiGrayGain + 1;
				}
			} 			
		}

		RoiGrayGain = 0;
		UseErosion = 0;
		RoiGray = RoiGray * (TrLi.RoiGrayGain / 5);
		for (size_t i = 0; i < TrLi.UseErosion; i++)
		{
			int erosion_type;
			int erosion_elem = 2;
			int erosion_size = 2;
			if (erosion_elem == 0){ erosion_type = MORPH_RECT; }
			else if (erosion_elem == 1){ erosion_type = MORPH_CROSS; }
			else if (erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }
			Mat element = getStructuringElement(erosion_type, Size(2 * erosion_size + 1, 2 * erosion_size + 1), Point(erosion_size, erosion_size));
			erode(RoiGray, RoiGray, element);
		}

		findContours();

		
		if ((E1V > 10) & (MaxColorAverage  < 600)) { TrLi.E1 = 1; TrLi.E1V_CENTER = E1V_CENTER; }
		if ((E2V > 10) & (MaxColorAverage  < 600)) { TrLi.E2 = 1; TrLi.E2V_CENTER = E2V_CENTER; }
		if ((E3V > 10) & (MaxColorAverage  < 600)) { TrLi.E3 = 1; TrLi.E3V_CENTER = E3V_CENTER; }
		if ((E4V > 10) & (MaxColorAverage  < 600)) { TrLi.E4 = 1; TrLi.E4V_CENTER = E4V_CENTER; }
		if ((E5V > 10) & (MaxColorAverage  < 600)) { TrLi.E5 = 1; TrLi.E5V_CENTER = E5V_CENTER; }

		TrLi.E1V = E1V;
		TrLi.E2V = E2V;
		TrLi.E3V = E3V;
		TrLi.E4V = E4V;
		TrLi.E5V = E5V;

		//Устанавливаем соответствие зон детекции по видео и модуля контроля силовых линий 
		/*ShowGraph = 3 это тестово статистическое состояние.
		  В этом состоянии вывод данных идёт только на картинку*/
		if (ShowGraph == 3)
		{
			if (TrLi.E1Color != 0){	TrLi.E1Tcl = state.getstate(TrLi.E1TclChannel); }
			if (TrLi.E2Color != 0){ TrLi.E2Tcl = state.getstate(TrLi.E2TclChannel); }
			if (TrLi.E3Color != 0){	TrLi.E3Tcl = state.getstate(TrLi.E3TclChannel); }
			if (TrLi.E4Color != 0){	TrLi.E4Tcl = state.getstate(TrLi.E4TclChannel); }
			if (TrLi.E5Color != 0){	TrLi.E5Tcl = state.getstate(TrLi.E5TclChannel); }
			
		}
		else
		{
			if (TrLi.E1Color != 0) 	{state.setstate(TrLi.E1TclChannel, TrLi.E1);}
			if (TrLi.E2Color != 0)	{state.setstate(TrLi.E2TclChannel, TrLi.E2);}
			if (TrLi.E3Color != 0)	{state.setstate(TrLi.E3TclChannel, TrLi.E3);}
			if (TrLi.E4Color != 0)	{state.setstate(TrLi.E4TclChannel, TrLi.E4);}
			if (TrLi.E5Color != 0)	{state.setstate(TrLi.E5TclChannel, TrLi.E5);}
		}

		hconcat(Color288x50, RoiHSV, joinImg);
		cvtColor(RoiGray, RoiGray, CV_GRAY2RGB);
		hconcat(joinImg, RoiGray, joinImg);
		hconcat(joinImg, RoiContours, joinImg);
		empty = Mat(pict_height, pict_width - 200, CV_8UC3) * 0;
		hconcat(joinImg, empty, joinImg);
		
		empty = Mat(96, pict_width, CV_8UC3) * 0;
		
		vconcat(joinImg, empty, joinImg);	 
		DrawDataToDebugPic();				 
		cvtColor(joinImg, joinImg, CV_BGR2YUV);
		outgrab.frame_meta->Grabbed = 0;
		for (int o = 0; o<pict_width*(pict_height + 96) / 2; o++)
		{
			outgrab.videobuf[o * 4] = joinImg.data[o * 6];
			outgrab.videobuf[o * 4 + 1] = (u_int8_t)((joinImg.data[o * 6 + 2] + joinImg.data[o * 6 + 5]) / 2);
			outgrab.videobuf[o * 4 + 2] = joinImg.data[o * 6 + 3];
			outgrab.videobuf[o * 4 + 3] = (u_int8_t)((joinImg.data[o * 6 + 1] + joinImg.data[o * 6 + 4]) / 2);
		}			
		outgrab.frame_meta->NFrame++;
		outgrab.frame_meta->Grabbed = 1;		
	}
	return 0;
}
