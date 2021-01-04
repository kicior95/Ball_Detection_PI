#include <iostream>
#include <chrono>
#include <stdio.h>
#include <cstdio>
#include <memory>
#include <string>
#include <array>
#include <string>


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


#include <boost/tuple/tuple.hpp>
#include "matplotlibcpp.h"


using namespace std;
using namespace cv;


namespace plt = matplotlibcpp;


struct Vector2D {
    float x;
    float y;
};



enum APROX_MODE {
    RECTANGLE,
    ARC
};




static APROX_MODE Mode = RECTANGLE;
static Vector2D ImageAxisX;
static Vector2D ImageAxisY;
static Vector2D Joint_01;
static Vector2D Joint_02;
static Vector2D Joint_03;
static Vector2D Ball;
static Vector2D BallPosImage;

float Pos_X_Set = 50;
float Pos_Y_Set = 50;
int mode = 1;

int MaxAreaContourId(vector <vector<cv::Point>> contours);
string RunCMD(const char* cmd);

Vector2D CircleCenter(Vector2D Point01, Vector2D Point02, Vector2D Point03);

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

int main()
{

    // >>>>>>>>>> Inicjalizacja stałych

    ImageAxisX.x = 100;
    ImageAxisX.y = 0;
    ImageAxisY.x = 0;
    ImageAxisY.y = 100;

    Joint_01.x = 103;
    Joint_01.y = 131;
    Joint_02.x = 122;
    Joint_02.y = 550;
    Joint_03.x = 463;
    Joint_03.y = 320;
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
    float RotMainAxis = AngleBetweenVector(ImageAxisX, MainAxis);
    // <<<<<<<<<< Inicjalizacja stałych



    // >>>>>>>>>> Przygotowanie wykresow
    std::vector<double>  PosX_v(400,0);
    std::vector<double>  SetPosX_v(400, Pos_Y_Set);
    std::vector<double>  RotX_v(400,0);

    std::vector<double>  PosY_v(400,0);
    std::vector<double>  SetPosY_v(400, Pos_X_Set);
    std::vector<double>  RotY_v(400,0);

    std::vector<double>  PosZ_v(400,0);
    std::vector<double>  SetPosZ_v(400, Pos_Y_Set);
    std::vector<double>  RotZ_v(400,0);

    std::vector<double> t(400);
    for(int i=0;i<t.size();i++) {
        t[i] = i;
    }





        //Wykres 01
        plt::figure_size(480, 320);
        plt::title("Pozycja X");

        plt:: subplot(2,1,1);
        plt:: Plot plot10("Pos01", t, PosX_v);
        plt:: Plot plot12("Set01", t, SetPosX_v);
        plt::grid(1);
        plt::legend();
        if(mode == 1) {
             plt::ylim(-250, 250);
        } else if (mode == 0) {
             plt::ylim(-250, 250);
        }



        plt:: subplot(2,1,2);
        plt:: Plot plot11("Rot01", t, RotY_v);
        plt::grid(1);
        plt::legend();

        if(mode == 1) {
             plt::ylim(-35, 35);
        } else if (mode == 0) {
             plt::ylim(-12, 12);
        }

        //Wykres 02
        plt::figure_size(480, 320);
        plt::title("Pozycja Y");

        plt:: subplot(2,1,1);
        plt:: Plot plot20("Pos02", t, PosY_v);
        plt:: Plot plot22("Set02", t, SetPosY_v);
        plt::grid(1);
        plt::legend();
        plt::ylim(-250, 250);


        plt:: subplot(2,1,2);
        plt:: Plot plot21("Rot02", t, RotX_v);
        plt::grid(1);
        plt::legend();
        if(mode == 1) {
             plt::ylim(-35, 35);
        } else if (mode == 0) {
             plt::ylim(-12, 12);
        }

        //Wykres 03
        plt::figure_size(480, 320);
        plt::title("Pozycja 03");

        plt:: subplot(2,1,1);
        plt:: Plot plot30("Pos03", t, PosZ_v);
        plt:: Plot plot32("Set03", t, SetPosZ_v);
        plt::grid(1);
        plt::legend();
        plt::ylim(-250, 250);


        plt:: subplot(2,1,2);
        plt:: Plot plot31("Rot03", t, RotZ_v);
        plt::grid(1);
        plt::legend();
        if(mode == 1) {
             plt::ylim(-35, 35);
        } else if (mode == 0) {
             plt::ylim(-12, 12);
        }


     int plot_refresh = 0;
    // <<<<<<<<<< Przygotowanie wykresow





    // >>>>>>>>>> Komunikacja UART z STM32
    int UART_Iteration = 0;
    char UART_Data_Recived[54];
    UART_Data_Recived[54] = '\0';
    char UART_Data_Transmited[15] = "1000 1001 1002";


    //Otwarcie portu komunikacyjnego
    int fd;
    if ((fd = serialOpen ("/dev/ttyUSB0", 115200)) < 0) {
        fprintf (stderr, "Błąd otwarcia urządzenia: %s\n", strerror (errno)) ;
        //return 1 ;
    }

    //Ustawienie parametrów komunikacji
    struct termios options ;
    tcgetattr(fd, &options);   // Odczytanie obecnych ustawień
    options.c_cflag &= static_cast<uint>(~CSIZE) ;  // Wyczyszczenie bitów
    options.c_cflag |= static_cast<uint>(CS8);     // Komunukacja 8 bitowa
    options.c_cflag &= static_cast<uint>(~PARENB);  // Brak bitu parzystości
    options.c_cflag &= static_cast<uint>(~CSTOPB);
    tcsetattr(fd,0,&options); // Set new options


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
    RunCMD("v4l2-ctl -d /dev/video0 -c auto_exposure=0 -c exposure_time_absolute=120 -c iso_sensitivity_auto=1 -c iso_sensitivity=4 -c power_line_frequency=1");
    cap.set(CAP_PROP_FRAME_WIDTH , 640);
    cap.set(CAP_PROP_FRAME_WIDTH , 480);
    cap.set(CAP_PROP_FRAME_WIDTH , 480);

    //Uruchomienie urzadzenia
    if (!cap.isOpened()) {
        cout<<"Błąd otwarcia kamerki"<<endl;
        return -1;
    }
    cout << "Kamera włączona" <<endl;
    // <<<<<<<<<< Przechwytywanie obrazu z kamerki

    // >>>>>>>>>> Utworzenie okna
    string Okno_01 = "Okno01";
    namedWindow(Okno_01, WINDOW_AUTOSIZE);
    // <<<<<<<<<< Utworzenie okna

    // >>>>>>>>>> Cykl pracy urządzenia
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    while(1) {
        // >>>>>>>>>> Przechwytywanie obrazu



        while(cap.isOpened()) {
            cap.read(src);


            if (src.empty()) {
                cout<< "Błąd przechwytu obrazu"<<endl;
                break;
            }
            // <<<<<<<<<< Przechwytywanie obrazu

            // >>>>>>>>>> Przetwarzanie obrazu
            //Obrót obrazu
            Mat src_rot;
            src.copyTo(src_rot);
            transpose(src_rot,src_rot);
            flip(src_rot,src_rot,ROTATE_90_COUNTERCLOCKWISE);

            // Rozmycie obrazu
            Mat src_blur;
            GaussianBlur(src_rot, src_blur, Size(5, 5), 3.0, 3.0);

            //Obraz w przestrzeni barw HSV
            Mat src_hsv;
            cvtColor(src_blur, src_hsv, COLOR_BGR2HSV);

            //Podział obrazu na kanały przestrzeni HSV
            vector<Mat> hsv_channels;
            split(src_hsv,hsv_channels);

            //Progowanie obrazu
            Mat hsv_binnary;
            inRange(src_hsv, Scalar(30, 100, 80) ,Scalar(75, 255, 255), hsv_binnary);

            //Operacja morfologicznego zamknięcia
            Mat binnary_erode;
            Mat binnary_dilate;

            erode(hsv_binnary, binnary_erode, getStructuringElement(MORPH_ELLIPSE, Size(3, 3))); //Operacja erozji morfologicznej
            dilate(binnary_erode, binnary_dilate, getStructuringElement(MORPH_ELLIPSE, Size(3, 3))); //Operacja dylatacji morfologicznej


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

                        cout<<center.x <<"   "<<center.y<<endl;
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


                //Wyswietlenie wykrytego konturu
                drawContours(src_rot, DetectBalls, Contour_Id, Kolor_02, 1);
                //Wyswietlenie prostokąta opisanego na konturze
                rectangle(src_rot, DetectBallsBox[static_cast<uint>(Contour_Id)], Kolor_02, 2);
                //Wyrysowanie srodka okregu
                circle(src_rot, center, 2, Kolor_01, -1);
                //Wyrysowanie zlaczy platformy
                circle(src_rot, Point(Joint_01.x, Joint_01.y) , 2, Kolor_03, -1);
                circle(src_rot, Point(Joint_02.x, Joint_02.y) , 2, Kolor_03, -1);
                circle(src_rot, Point(Joint_03.x, Joint_03.y) , 2, Kolor_03, -1);
                //Wyrysowanie osi glownej
                line(src_rot,Point(FlipedVector(CenterOfPlatform).x, FlipedVector(CenterOfPlatform).y), Point(Joint_01.x, Joint_01.y), Kolor_04);
                //Wysysowanie srodka platformy
                circle(src_rot, Point(FlipedVector(CenterOfPlatform).x, FlipedVector(CenterOfPlatform).y) , 2, Kolor_03, -1);

                //Wyznaczenie polozenia srodka obiektu
                Pos_X = Ball.x;
                Pos_Y = Ball.y;

                // Wypisanie wyniku na ekranie
                stringstream sstr;
                sstr << "(" << static_cast<int>(Pos_X) << "," << static_cast<int>(Pos_Y) << ")";
                cout<<sstr.str()<<endl;
                putText(src_rot, sstr.str(),
                        Point(static_cast<int>(center.x + 3), static_cast<int>(center.y - 3)),
                        FONT_HERSHEY_SIMPLEX, 0.5, Kolor_01, 2);

            }
            // <<<<<<<<<<  Wyswietlanie wyników


            // >>>>>>>>>> Wysłanie danych do STM32

            sprintf(UART_Data_Transmited,"%04d %04d %04d %04d",static_cast<int>(Pos_X+1000), static_cast<int>(Pos_Y+1000),
                    static_cast<int>(Pos_X_Set+1000), static_cast<int>(Pos_Y_Set+1000));
            serialPrintf(fd,UART_Data_Transmited); // Wysyłanie wiadomości
            cout<<UART_Data_Transmited<<endl;
            // <<<<<<<<<< Wysłanie danych do STM32

            // >>>>>>>>>> Odbiór danych danych do STM32
            int itr = 0;
            while (serialDataAvail(fd) && itr <= 52) {
                UART_Data_Recived[itr] = static_cast<char>(serialGetchar(fd));
                itr++;
            }
            cout<<UART_Data_Recived<<endl;


            // >>>>>>>>>> Zapisanie danych do historii
            //PosX_v.erase(PosX_v.begin());
           // PosY_v.erase(PosY_v.begin());
            //PosX_v.push_back((double)Pos_X);
            //PosY_v.push_back((double)Pos_Y);

            char tempchar[6] = "00.00";

            // Zapisanie obecnej pozycji 01
            tempchar[0] = UART_Data_Recived[0];
            tempchar[1] = UART_Data_Recived[1];
            tempchar[2] = UART_Data_Recived[2];
            tempchar[3] = UART_Data_Recived[3];
            tempchar[4] = UART_Data_Recived[4];

            try {
                float temp =  atof(tempchar) - 500;
                PosX_v.erase(PosX_v.begin());
                PosX_v.push_back((double)temp);
            } catch (const std::runtime_error& e) {

            }
             // Zapisanie obecnego sterowania 01
            tempchar[0] = UART_Data_Recived[6];
            tempchar[1] = UART_Data_Recived[7];
            tempchar[2] = UART_Data_Recived[8];
            tempchar[3] = UART_Data_Recived[9];
            tempchar[4] = UART_Data_Recived[10];

            try {
                float temp =  atof(tempchar) - 500;
                RotX_v.erase(RotX_v.begin());
                RotX_v.push_back((double)temp);
            } catch (const std::runtime_error& e) {

            }

            // Zapisanie obecnej pozycji 02
            tempchar[0] = UART_Data_Recived[12];
            tempchar[1] = UART_Data_Recived[13];
            tempchar[2] = UART_Data_Recived[14];
            tempchar[3] = UART_Data_Recived[15];
            tempchar[4] = UART_Data_Recived[16];

            try {
                float temp =  atof(tempchar) - 500;
                PosY_v.erase(PosY_v.begin());
                PosY_v.push_back((double)temp);
            } catch (const std::runtime_error& e) {

            }

            // Zapisanie obecnego sterowania 02
            tempchar[0] = UART_Data_Recived[18];
            tempchar[1] = UART_Data_Recived[19];
            tempchar[2] = UART_Data_Recived[20];
            tempchar[3] = UART_Data_Recived[21];
            tempchar[4] = UART_Data_Recived[22];

            try {
                float temp =  atof(tempchar) - 500;
                RotY_v.erase(RotY_v.begin());
                RotY_v.push_back((double)temp);
            } catch (const std::runtime_error& e) {

            }

            // Zapisanie obecnej pozycji 03
            tempchar[0] = UART_Data_Recived[24];
            tempchar[1] = UART_Data_Recived[25];
            tempchar[2] = UART_Data_Recived[26];
            tempchar[3] = UART_Data_Recived[27];
            tempchar[4] = UART_Data_Recived[28];

            try {
                float temp =  atof(tempchar) - 500;
                PosZ_v.erase(PosZ_v.begin());
                PosZ_v.push_back((double)temp);
            } catch (const std::runtime_error& e) {

            }

            // Zapisanie obecnego sterowania 03
            tempchar[0] = UART_Data_Recived[30];
            tempchar[1] = UART_Data_Recived[31];
            tempchar[2] = UART_Data_Recived[32];
            tempchar[3] = UART_Data_Recived[33];
            tempchar[4] = UART_Data_Recived[34];

            try {
                float temp =  atof(tempchar) - 500;
                RotZ_v.erase(RotZ_v.begin());
                RotZ_v.push_back((double)temp);
            } catch (const std::runtime_error& e) {

            }

            // Zapisanie zadanego sterowania 01
            tempchar[0] = UART_Data_Recived[36];
            tempchar[1] = UART_Data_Recived[37];
            tempchar[2] = UART_Data_Recived[38];
            tempchar[3] = UART_Data_Recived[39];
            tempchar[4] = UART_Data_Recived[40];

            try {
                float temp =  atof(tempchar) - 500;
                SetPosX_v.erase(SetPosX_v.begin());
                SetPosX_v.push_back((double)temp);
            } catch (const std::runtime_error& e) {

            }

            // Zapisanie zadanego sterowania 02
            tempchar[0] = UART_Data_Recived[42];
            tempchar[1] = UART_Data_Recived[43];
            tempchar[2] = UART_Data_Recived[44];
            tempchar[3] = UART_Data_Recived[45];
            tempchar[4] = UART_Data_Recived[46];

            try {
                float temp =  atof(tempchar) - 500;
                SetPosY_v.erase(SetPosY_v.begin());
                SetPosY_v.push_back((double)temp);
            } catch (const std::runtime_error& e) {

            }

            // Zapisanie zadanego sterowania 03
            tempchar[0] = UART_Data_Recived[48];
            tempchar[1] = UART_Data_Recived[49];
            tempchar[2] = UART_Data_Recived[50];
            tempchar[3] = UART_Data_Recived[51];
            tempchar[4] = UART_Data_Recived[52];

            try {
                float temp =  atof(tempchar) - 500;
                SetPosZ_v.erase(SetPosZ_v.begin());
                SetPosZ_v.push_back((double)temp);
            } catch (const std::runtime_error& e) {

            }

        // <<<<<<<<<< Zapisanie danych do historii
            plot_refresh ++;

            //Wyczyszczenie otrzymanej tablicy
            itr = 0;
            while(itr <= 33) {
                UART_Data_Recived[itr] = 0;
                itr ++;
            }

            // <<<<<<<<<< Odbiór danych danych do STM32



            // >>>>>>>>>> GUI: Oczekiwanie na akcje uzytkownika

            //Wyznaczenie predkosci akwiztcji danych
            UART_Iteration++;
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            auto diff = end - begin;
            cout <<"Fps: " <<1000 * UART_Iteration / chrono::duration_cast<std::chrono::milliseconds>(diff).count() << endl;
            //Wyrysowanie wykresow
            if(plot_refresh>350) {
                if(mode == 0) {
                    plot10.update(t,PosX_v);
                    plot20.update(t,PosY_v);

                    plot11.update(t,RotX_v);
                    plot21.update(t,RotY_v);

                    plot12.update(t,SetPosX_v);
                    plot22.update(t,SetPosY_v);
                } else if (mode == 1) {
                    plot10.update(t,PosX_v);
                    plot20.update(t,PosY_v);
                    plot30.update(t,PosZ_v);

                    plot11.update(t,RotX_v);
                    plot21.update(t,RotY_v);
                    plot31.update(t,RotZ_v);

                    plot12.update(t,SetPosX_v);
                    plot22.update(t,SetPosY_v);
                    plot32.update(t,SetPosZ_v);
                }


                plot_refresh = 0;
                plt::pause(0.000001);
            }


            //Wyswietlenie obrazu
            imshow(Okno_01,src_rot);
            // >>>>>>>>>> DIAGNOSTYKA
            //imshow("Hue",hsv_channels[0]);
            //imshow("Saturation",hsv_channels[2]);
            //imshow("Value",hsv_channels[2]);

            // imshow("Threshold",hsv_binnary);
            //imshow("Erode",binnary_erode);
            //imshow("Dilate",binnary_dilate);
            // <<<<<<<<<< DIAGNOSTYKA

            // Wcisniecie przyciku zarzymuje cykl
            if (waitKey(5) >= 0)
                return 0;
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
