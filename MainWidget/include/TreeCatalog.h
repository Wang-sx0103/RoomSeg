/*****************************************************************//**
 * \file   TreeCatalog.h
 * \brief  
 * 
 * \author AlexW
 * \date   December 2023
 *********************************************************************/
#pragma once

#include <QList>
#include <QMap>
#include <QString>

class TreeCatalog
{
public:
    TreeCatalog();
    TreeCatalog(const TreeCatalog&);
    TreeCatalog& operator=(const TreeCatalog&);
    QList<QString>*& operator[](const QString);
    void insert(const QString, QList<QString>*);
    void remove(const QString);
    QString removeSubNode(const QString, const int);
    QString getSubNodeName(const QString, const int);
    QList<QString> keys();
    QList<QList<QString>*> values();
    void clear(void);
    //void pushBack(const QString);
    
    ~TreeCatalog();

private:
    QMap<QString, QList<QString>*> mmCatalog;
};
