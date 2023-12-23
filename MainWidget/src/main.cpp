/*****************************************************************//**
 * \file   main.cpp
 * \brief  
 * 
 * \author AlexW
 * \date   December 2023
 *********************************************************************/
#include "MainWidget.h"
#include <QtWidgets/QApplication>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWidget w;
    w.show();
    return a.exec();
}
