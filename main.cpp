#include <iostream>
#include <stdio.h>


//Bibl
#include <wiringPi.h>
#include <wiringSerial.h>
#include <termios.h>

//Bilioteki OpenCV
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <raspicam/raspicam_cv.h>


using namespace std;
using namespace cv;
using namespace raspicam;


int main()
{
    // >>>>>>>>>> Komunikacja UART z STM32
    int UART_Iteration = 0;
    char UART_Data_Recived[34] = "";
    char UART_Data_Transmited[15] = "1000 1001 1002";


    //Otwarcie portu komunikacyjnego
    int fd;
    if ((fd = serialOpen ("/dev/ttyUSB0", 115200)) < 0) {
        fprintf (stderr, "Błąd otwarcia urządzenia: %s\n", strerror (errno)) ;
        return 1 ;
    }

    //Ustawienie parametrów komunikacji
    struct termios options ;
    tcgetattr(fd, &options);   // Odczytanie obecnych ustawień
    options.c_cflag &= ~CSIZE ;  // Wyczyszczenie bitów
    options.c_cflag |= CS8;     // Komunukacja 8 bitowa
    options.c_cflag &= ~PARENB;  // Brak bitu parzystości
    options.c_cflag &= ~CSTOPB;
    tcsetattr(fd,0,&options); // Set new options


    if (wiringPiSetup () == -1) {
        fprintf (stdout, "Błąd otwarcia biblioteki: %s\n", strerror (errno)) ;
        return 1 ;
    }
    // <<<<<<<<<< Komunikacja UART z STM32

    // >>>>>>>>>> Przechwytywanie obrazu z kamerki
    VideoCapture cap;
    Mat src;
    cap.open(0);
cap.set(CAP_PROP_FRAME_WIDTH , 640);
cap.set(CAP_PROP_FRAME_WIDTH , 480);
cap.set(CAP_PROP_FPS , 25);
cap.set(CAP_PROP_EXPOSURE, 0);

   if (!cap.isOpened()) {
        cout<<"Błąd otwarcia kamerki"<<endl;
        return -1;
    }
    cout << "Kamera włączona" <<endl;
    // <<<<<<<<<< Przechwytywanie obrazu z kamerki

    // >>>>>>>>>> Utworzenie okna
    string Okno_01 = "Okno01";
    int Value_Min = 0;
    int Value_Max = 0;
    namedWindow(Okno_01, WINDOW_AUTOSIZE);
    createTrackbar( "Zmienna 01", Okno_01, &Value_Min, 255000);
    createTrackbar( "Zmienna 02", Okno_01, &Value_Max, 255);
    // <<<<<<<<<< Utworzenie okna

    // >>>>>>>>>> Cykl pracy urządzenia
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

            // Rozmycie obrazu
            Mat src_blur;
            GaussianBlur(src, src_blur, Size(5, 5), 3.0, 3.0);

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

             //Analiza konturów
             vector<vector<Point> > balls;
             Point2f center;
             float radius;
             vector<Rect> ballsBox;

             for (size_t i = 0; i < contours.size(); i++) {
                 Rect bBox;
                 bBox = boundingRect(contours[i]);
                 float ratio = (float) bBox.width / (float) bBox.height;
                 if (ratio > 1.0f)
                     ratio = 1.0f / ratio;

                 // Searching for a bBox almost square
                 if (ratio > 0.4 && bBox.area() >= 300) {
                     Point2f center;
                     float radius;
                     balls.push_back(contours[i]);
                     ballsBox.push_back(bBox);
                 }
             }

                 // >>>>> Detection result
                                   for (size_t i = 0; i < balls.size(); i++) {
                                       drawContours(src, balls, i, (20,150,20), 1);
                                       rectangle(src, ballsBox[i], (0,255,0), 2);

                                       //Point center;
                                       center.x = ballsBox[i].x + ballsBox[i].width / 2;
                                       center.y = ballsBox[i].y + ballsBox[i].height / 2;
                                       circle(src, center, 2, (20,150,20), -1);

                                      stringstream sstr;
                                       sstr << "(" << center.x-320 << "," << center.y-240 << ")";
                                       putText(src, sstr.str(),
                                                  Point(center.x + 3, center.y - 3),
                                                  FONT_HERSHEY_SIMPLEX, 0.5, (20,150,20), 2);
                                   }
                                   // <<<<< Detection result

            // <<<<<<<<<< Przetwarzanie obrazu

            // >>>>>>>>>> Wysłanie danych do STM32
            cout << "Wysylanie: "<<UART_Iteration<<endl;
            serialPrintf(fd,UART_Data_Transmited); // Wysyłanie wiadomości
            delay(10);
            // <<<<<<<<<< Wysłanie danych do STM32

            // >>>>>>>>>> Odbiór danych danych do STM32
            int itr = 0;
            while (serialDataAvail(fd) && itr < 35) {
                UART_Data_Recived[itr] = serialGetchar(fd);
                itr++;
            }
            cout <<UART_Data_Recived<<endl;

            //Wyczyszczenie otrzymanej tablicy
            itr = 0;
            if(itr < 35) {
                UART_Data_Recived[itr] = 0;
                itr ++;
            }
            UART_Iteration++;
            // <<<<<<<<<< Odbiór danych danych do STM32



            // >>>>>>>>>> GUI: Oczekiwanie na akcje uzytkownika

            //Wyswietlenie obrazu
            imshow(Okno_01,src);

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

    return 0;
}
