#include <iostream>
#include<fstream>

#include <chrono>
#include <time.h>

#include <stdio.h>
#include <cstdio>

#include <memory>
#include <string>
#include <array>
#include <string>
#include <list>



//Biblioteki transmisji szeregowej
#include <wiringPi.h>
#include <wiringSerial.h>
#include <termios.h>

//Bilioteki OpenCV
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>



using namespace std;
using namespace cv;



struct Vector2D {
    float x;
    float y;
};


enum APROX_MODE {
    RECTANGLE,
    ARC
};

enum CHART_MODE {
    ON,
    OFF,
    SAVE
};

static APROX_MODE Mode = RECTANGLE;
static Vector2D ImageAxisX;
static Vector2D ImageAxisY;
static Vector2D Joint_01;
static Vector2D Joint_02;
static Vector2D Joint_03;
static Vector2D Ball;
static Vector2D BallPosImage;

float Pos_X_Set = 0;
float Pos_Y_Set = 0;
int mode = 0;
int zapis = 0;

int MaxAreaContourId(vector <vector<cv::Point>> contours);
string RunCMD(const char* cmd);
Vector2D CircleCenter(Vector2D Point01, Vector2D Point02, Vector2D Point03);
Vector2D SubtractVector(Vector2D Wektor_01, Vector2D Wektor_02);
Vector2D ReversVector(Vector2D Wektor_01);
Vector2D ScalarVectorMultiplication(Vector2D Wektor, float Skalar);
float VectorScalar(Vector2D Wektor_01, Vector2D  Wektor_02);
float VectorNorm(Vector2D Wektor);
float AngleBetweenVector(Vector2D Wektor_01, Vector2D  Wektor_02);
Vector2D FlipedVector(Vector2D Wektor_01);
Vector2D CalcPos(Vector2D Ball_Pos_Image, Vector2D  CircleCenter, float AngleOfMainVector);


void wywolanieMyszki(int zdarzenie, int x, int y, int flagi, void* parametr);


int main() {
    
    
    
    // >>>>>>>>>> Inicjalizacja stałych
    
    ImageAxisX.x = 100;
    ImageAxisX.y = 0;
    ImageAxisY.x = 0;
    ImageAxisY.y = 100;
    
    
    Joint_01.x = 142;
    Joint_01.y = 363;
    Joint_02.x = 325;
    Joint_02.y = 24;
    Joint_03.x = 530;
    Joint_03.y = 351;
    Ball.x  = 0;
    Ball.y  = 0;
    BallPosImage.x = 0;
    BallPosImage.y = 0;
    
    
    // Wyznaczenie srodka platformy
    Vector2D CenterOfPlatform;
    CenterOfPlatform = CircleCenter(FlipedVector(Joint_01), FlipedVector(Joint_02), FlipedVector(Joint_03));
    
    //Wyznaczenie glownej osi platformy
    Vector2D MainAxis;
    MainAxis = ReversVector(SubtractVector(CenterOfPlatform, FlipedVector(Joint_01)));
    
    //Wyznaczenie katu obrotu osi platformy
    float RotMainAxis = -AngleBetweenVector(ImageAxisX, MainAxis);
    // <<<<<<<<<< Inicjalizacja stałych
    
    
    
    // >>>>>>>>>> Komunikacja UART z STM32   
    char UART_Data_Recived[53];
    UART_Data_Recived[52] = '\0';
    char UART_Data_Transmited[20] = "1000 1000 1000 1000";
    
    //Otwarcie portu komunikacyjnego
    int fd;
    if ((fd = serialOpen ("/dev/ttyUSB0", 19200)) < 0) {
        fprintf (stderr, "Błąd otwarcia urządzenia: %s\n", strerror (errno)) ;
        //return 1 ;
    }
    
    //Ustawienie parametrów komunikacji
    struct termios options ;
    tcgetattr(fd, &options);                        // Odczytanie obecnych ustawień
    options.c_cflag &= static_cast<uint>(~CSIZE);   // Wyczyszczenie bitów
    options.c_cflag |= static_cast<uint>(CS8);      // Komunukacja 8 bitowa
    options.c_cflag &= static_cast<uint>(~PARENB);  // Brak bitu parzystości
    options.c_cflag &= static_cast<uint>(~CSTOPB);  // Ustawienie bitu stopu
    tcsetattr(fd,0,&options);                       // Zapisanie ustawien
    
    if (wiringPiSetup () == -1) {
        fprintf (stdout, "Błąd otwarcia biblioteki: %s\n", strerror(errno)) ;
        //return 1 ;
    }
    // <<<<<<<<<< Komunikacja UART z STM32
    
    // >>>>>>>>>> Przechwytywanie obrazu z kamerki
    // Inicjalizacja zmiennych
    VideoCapture cap;
    Mat src;
    cap.open(0);
    
    //Konfiguracja sprzetu
    //RunCMD("v4l2-ctl -d /dev/video0 -c auto_exposure=0 -c exposure_time_absolute=120 -c iso_sensitivity_auto=1 -c iso_sensitivity=4 -c power_line_frequency=1");
    RunCMD("v4l2-ctl --set-fmt-video=width=640,height=480,pixelformat=2 --set-parm=60");
    RunCMD("v4l2-ctl -d /dev/video0 -c auto_exposure=0 -c white_balance_auto_preset=1 -c exposure_time_absolute=80 -c iso_sensitivity_auto=0 -c iso_sensitivity=4 -c power_line_frequency=1");
    
    
    cap.set(CAP_PROP_FRAME_WIDTH , 640);
    cap.set(CAP_PROP_FRAME_HEIGHT , 480);
    cap.set(CAP_PROP_FPS,60);
    cap.set(CAP_PROP_BUFFERSIZE, 1);
    
    //Uruchomienie urzadzenia
    if (!cap.isOpened()) {
        cout<<"Błąd uruchomienia kamery"<<endl;
        return -1;
    }
    cout << "Kamera włączona" <<endl;
    // <<<<<<<<<< Przechwytywanie obrazu z kamerki
    
    // >>>>>>>>>> Utworzenie okna
    string Okno_01 = "Okno01";
    namedWindow(Okno_01, WINDOW_AUTOSIZE);
    setMouseCallback(Okno_01, wywolanieMyszki, NULL);
    // <<<<<<<<<< Utworzenie okna
    
    // >>>>>>>>>> Cykl pracy urządzenia
    static std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();
    static std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    
    while(1) {
        // >>>>>>>>>> Przechwytywanie obrazu      
        
        while(cap.isOpened()) {
            begin = std::chrono::high_resolution_clock::now();
            cap.read(src);
            
            if (src.empty()) {
                cout<< "Błąd przechwytu obrazu"<<endl;
                break;
            }
            // <<<<<<<<<< Przechwytywanie obrazu
            
            // >>>>>>>>>> Przetwarzanie obrazu
            // Rozmycie obrazu
            Mat src_blur;
            GaussianBlur(src, src_blur, Size(5, 5), 3.0, 3.0);
            
            //Obraz w przestrzeni barw HSV
            Mat src_hsv;
            cvtColor(src_blur, src_hsv, COLOR_BGR2HSV);
            
            //Progowanie obrazu
            Mat hsv_binnary;
            inRange(src_hsv, Scalar(30, 50, 80) ,Scalar(75, 255, 255), hsv_binnary);
            
            //Operacja morfologicznego zamknięcia
            Mat binnary_erode;
            Mat binnary_dilate;
            
            //Operacja erozji morfologicznej
            erode(hsv_binnary, binnary_erode,
                  getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
            
            //Operacja dylatacji morfologicznej
            dilate(binnary_erode, binnary_dilate, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
            
            //Znalezienie konturów
            vector<vector<Point> > contours;
            findContours(binnary_dilate, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
            
            //Jeżeli znaleziono kontury
            vector<vector<Point>> DetectBalls;
            Point2f center;
            vector<Rect> DetectBallsBox;
            Rect Rectangle;
            int Contour_Id = -1;
            float Pos_X = 0;
            float Pos_Y = 0;
            
            if(contours.size() > 0) {
                
                //Analiza konturów
                if(Mode == RECTANGLE) {
                    //Wyszukiwanie obiektów kwadratowych
                    for (size_t i = 0; i < contours.size(); i++) {
                        
                        //Opisanie prostokątu na konturze
                        Rectangle = boundingRect(contours[i]);
                        //Wyznaczenie wspołczynnika proporcjonalnosci dlugosci boków protokąta
                        double Rectangl_Ratio = static_cast<double>(Rectangle.width) / static_cast<double>(Rectangle.height);
                        if (Rectangl_Ratio > 1)
                            Rectangl_Ratio = 1/Rectangl_Ratio;
                        
                        // Poszukiwanie prostokątów o zadanych proporcjach
                        if (Rectangl_Ratio > 0.6 && Rectangle.area() > 600 ) {
                            DetectBalls.push_back(contours[i]);
                            DetectBallsBox.push_back(Rectangle);
                        }
                    }
                    if(DetectBalls.size() > 0) {
                        Contour_Id = MaxAreaContourId(DetectBalls);
                        //Srodek geometryczny detalu na obrazie;
                        center.x = DetectBallsBox[static_cast<uint>(Contour_Id)].x + DetectBallsBox[static_cast<uint>(Contour_Id)].width / 2;
                        center.y = DetectBallsBox[static_cast<uint>(Contour_Id)].y + DetectBallsBox[static_cast<uint>(Contour_Id)].height / 2;
                        
                        //Wyznaczenie polozenia srodka obiektu
                        BallPosImage.x = center.x;
                        BallPosImage.y = center.y;
                        Ball = CalcPos(BallPosImage, CenterOfPlatform, RotMainAxis);
                    }  
                }
                
                if(Mode == ARC) {
                    for(size_t i = 0; i < contours.size(); i++) {
                        // Wyznaczenie okrągłości
                        double area = contourArea(contours[static_cast<uint>(i)]);
                        double arclength = arcLength(contours[static_cast<uint>(i)], true);
                        double circularity = 4 * CV_PI * area / (arclength * arclength);
                        Rectangle = boundingRect(contours[static_cast<uint>(i)]);
                        if(circularity > 0.6)
                        {
                            DetectBalls.push_back(contours[static_cast<uint>(i)]);
                            DetectBallsBox.push_back(Rectangle);
                            
                        }
                    }
                    if(DetectBalls.size() > 0) {
                        Contour_Id = MaxAreaContourId(DetectBalls);
                        //Wyznaczenie środka
                        Moments mu = moments(contours[static_cast<uint>(Contour_Id)], false);
                        center.x = static_cast<float>(mu.m10 / mu.m00); // Wspolrzedna X
                        center.y = static_cast<float>(mu.m01 / mu.m00); // Wspolrzedna Y
                        
                        //Wyznaczenie polozenia srodka obiektu
                        BallPosImage.x = center.x;
                        BallPosImage.y = center.y;
                        Ball = CalcPos(BallPosImage, CenterOfPlatform, RotMainAxis);
                        
                    }
                    
                }
                
            }
            
            // <<<<<<<<<< Przetwarzanie obrazu
            
            
            // >>>>>>>>>> Wyswietlanie wyników
            if(Contour_Id >= 0) {
                //Definicja kolorów
                Scalar Kolor_01 = Scalar(20,150,20);
                Scalar Kolor_02 = Scalar(0, 255, 0);
                Scalar Kolor_03 = Scalar(255, 255, 0);
                Scalar Kolor_04 = Scalar(0, 255, 255);
                Scalar Kolor_05 = Scalar(128,0,0);
                
                
                //Wyswietlenie wykrytego konturu
                drawContours(src, DetectBalls, Contour_Id, Kolor_02, 1);
                if(zapis == 1) imwrite("Contours.png", src);
                //Wyswietlenie prostokąta opisanego na konturze
                rectangle(src, DetectBallsBox[static_cast<uint>(Contour_Id)], Kolor_02, 2);
                //Wyrysowanie srodka okregu
                circle(src, center, 2, Kolor_01, -1);
                //Wyrysowanie zlaczy platformy
                circle(src, Point(Joint_01.x, Joint_01.y) , 2, Kolor_03, -1);
                circle(src, Point(Joint_02.x, Joint_02.y) , 2, Kolor_03, -1);
                circle(src, Point(Joint_03.x, Joint_03.y) , 2, Kolor_03, -1);
                //Wyrysowanie osi glownej
                line(src,Point(FlipedVector(CenterOfPlatform).x, FlipedVector(CenterOfPlatform).y), Point(Joint_01.x, Joint_01.y), Kolor_04);
                //Wysysowanie srodka platformy
                circle(src, Point(FlipedVector(CenterOfPlatform).x, FlipedVector(CenterOfPlatform).y) , 2, Kolor_03, -1);
                
                //Wyznaczenie polozenia srodka obiektu
                Pos_X = Ball.x;
                Pos_Y = Ball.y;
                
                // Wypisanie wyniku na ekranie
                stringstream sstr;
                sstr << "(" << static_cast<int>(Pos_X) << "," << static_cast<int>(Pos_Y) << ")";
                
                putText(src, sstr.str(),
                        Point(static_cast<int>(center.x + 3), static_cast<int>(center.y - 3)),
                        FONT_HERSHEY_SIMPLEX, 0.5, Kolor_05, 2);
                
            }
            
            //Wyswietlenie obrazu
            imshow(Okno_01,src);           
            // <<<<<<<<<<  Wyswietlanie wyników
            
            
            // >>>>>>>>>> Wysłanie danych do STM32
            sprintf(UART_Data_Transmited,"%04d %04d %04d %04d",static_cast<int>(Pos_X+1000), static_cast<int>(Pos_Y+1000),
                    static_cast<int>(Pos_X_Set+1000), static_cast<int>(Pos_Y_Set+1000));
            
            // Wysyłanie wiadomości
            serialPrintf(fd,UART_Data_Transmited); 
            // <<<<<<<<<< Wysłanie danych do STM32
            
            // >>>>>>>>>> Odbiór danych danych do STM32
            int itr = 0;
            while (serialDataAvail(fd) && itr <= 52) {
                UART_Data_Recived[itr] = static_cast<char>(serialGetchar(fd));
                itr++;
            }
            
            //Wyczyszczenie otrzymanej tablicy
            itr = 0;
            while(itr <= 33) {
                UART_Data_Recived[itr] = 0;
                itr ++;
            }
            // <<<<<<<<<< Odbiór danych danych do STM32
            
            // >>>>>>>>>> GUI: Oczekiwanie na akcje uzytkownika          
            //Wyznaczenie predkosci akwiztcji danych
            end = std::chrono::high_resolution_clock::now();
            auto TOTAL_TIME = end-begin;
            cout <<"TOTAL_TIME: " << chrono::duration_cast<std::chrono::microseconds>(TOTAL_TIME).count() << endl;
            
            //Wcisniecie przyciku zarzymuje cykl
            int Button_Number = waitKey(1);
            
            if (Button_Number == 27) {
                return 0;
            }                      
        }
        // >>>>>>>>>> GUI: Oczekiwanie na akcje uzytkownika
        
    }
    // <<<<<<<<<< Cykl pracy urządzenia    
}


int MaxAreaContourId(vector <vector<cv::Point>> contours) {
    double Max_Contour_Area = 0;
    int Contour_Id = 0;
    for (int i = 0; i < static_cast<int>(contours.size()); i++) {
        double Contour_Area = contourArea(contours.at(static_cast<uint>(i)));
        if (Contour_Area > Max_Contour_Area) {
            Max_Contour_Area = Contour_Area;
            Contour_Id = i;
        }
    }
    return Contour_Id;
}

string RunCMD(const char* cmd) {
    array<char, 128> buffer;
    string result;
    unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}



Vector2D CircleCenter(Vector2D Point01, Vector2D Point02, Vector2D Point03) {
    Vector2D Center;
    Center.x = static_cast<float>(0.5f * ((Point02.x * Point02.x * Point03.y + Point02.y * Point02.y * Point03.y - Point01.x * Point01.x * Point03.y + Point01.x * Point01.x * Point02.y - Point01.y * Point01.y * Point03.y + Point01.y * Point01.y * Point02.y + Point01.y * Point03.x * Point03.x + Point01.y * Point03.y * Point03.y - Point01.y * Point02.x * Point02.x - Point01.y * Point02.y * Point02.y - Point02.y * Point03.x * Point03.x - Point02.y * Point03.y * Point03.y) / (Point01.y * Point03.x - Point01.y * Point02.x - Point02.y * Point03.x - Point03.y * Point01.x + Point03.y * Point02.x + Point02.y * Point01.x)));
    Center.y = static_cast<float>(0.5f * ((-Point01.x * Point03.x * Point03.x - Point01.x * Point03.y * Point03.y + Point01.x * Point02.x * Point02.x + Point01.x * Point02.y * Point02.y + Point02.x * Point03.x * Point03.x + Point02.x * Point03.y * Point03.y - Point02.x * Point02.x * Point03.x - Point02.y * Point02.y * Point03.x + Point01.x * Point01.x * Point03.x - Point01.x * Point01.x * Point02.x + Point01.y * Point01.y * Point03.x - Point01.y * Point01.y * Point02.x) / (Point01.y * Point03.x - Point01.y * Point02.x - Point02.y * Point03.x - Point03.y * Point01.x + Point03.y * Point02.x + Point02.y * Point01.x)));
    return Center;
}
Vector2D SubtractVector(Vector2D Wektor_01, Vector2D Wektor_02) {
    Vector2D results;
    results.x = Wektor_01.x - Wektor_02.x;
    results.y = Wektor_01.y - Wektor_02.y;
    return results;
}

Vector2D ReversVector(Vector2D Wektor_01) {
    Vector2D results;
    results.x = - Wektor_01.x;
    results.y = - Wektor_01.y;
    return results;
}

Vector2D ScalarVectorMultiplication(Vector2D Wektor, float Skalar) {
    Vector2D results;
    results.x = Skalar * Wektor.x;
    results.y = Skalar * Wektor.y;
    return results;
}

float VectorScalar(Vector2D Wektor_01, Vector2D  Wektor_02) {
    float results = Wektor_01.x * Wektor_02.x + Wektor_01.y * Wektor_02.y;
    return results;
}

float VectorNorm(Vector2D Wektor) {
    float VectorNorm = sqrt(pow(Wektor.x, 2) + pow(Wektor.y, 2));
    return VectorNorm;
}

float AngleBetweenVector(Vector2D Wektor_01, Vector2D  Wektor_02) {
    float results = acos(VectorScalar(Wektor_01, Wektor_02)/(VectorNorm(Wektor_01) * VectorNorm(Wektor_02)));
    return results;
}

Vector2D FlipedVector(Vector2D Wektor_01) {
    Vector2D results;
    results.x = Wektor_01.x;
    results.y = -Wektor_01.y;
    return results;
}


Vector2D CalcPos(Vector2D Ball_Pos_Image, Vector2D  CircleCenter, float AngleOfMainVector) {
    Vector2D temp = SubtractVector(FlipedVector(Ball_Pos_Image), CircleCenter);
    Vector2D results;
    results.x = cos(AngleOfMainVector) * temp.x + sin(AngleOfMainVector) * temp.y;
    results.y = -sin(AngleOfMainVector) * temp.x + cos(AngleOfMainVector) * temp.y;
    return results;
}

void wywolanieMyszki(int zdarzenie, int x, int y, int flagi, void* parametr) {
    if(zdarzenie == EVENT_LBUTTONDOWN) {
        cout<<x<<" "<<y<<endl;
        zapis = 1;
    }
}
