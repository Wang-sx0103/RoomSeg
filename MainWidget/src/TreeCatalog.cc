/*****************************************************************//**
 * \file   TreeCatalog.cc
 * \brief  
 * 
 * \author AlexW
 * \date   December 2023
 *********************************************************************/
#include "TreeCatalog.h"

#include <iostream>

TreeCatalog::TreeCatalog()
{
}
TreeCatalog::TreeCatalog(const TreeCatalog &cur)
{
    this->clear();
    for (QMap<QString, QList<QString>*>::const_iterator it = cur.mmCatalog.constBegin(); it != cur.mmCatalog.constEnd(); ++it)
    {
        //delete[] this->mmCatalog[it.key()];
        this->insert(it.key(), it.value());
    }
    //this->mmCatalog = cur.mmCatalog;
}

TreeCatalog& TreeCatalog::operator=(const TreeCatalog &cur)
{
    this->clear();
    for (QMap<QString, QList<QString>*>::const_iterator it = cur.mmCatalog.constBegin(); it != cur.mmCatalog.constEnd(); ++it)
    {
        //delete[] this->mmCatalog[it.key()];
        this->insert(it.key(), it.value());
    }
    return *this;
}

/**
* \brief
* \param[in]
*/
QList<QString>*& TreeCatalog::operator[](const QString& topNode)
{
    return this->mmCatalog[topNode];
}

void TreeCatalog::insert(const QString& topNode, QList<QString>* value)
{
    this->mmCatalog[topNode] = value;
}

void TreeCatalog::remove(const QString& topNode)
{
    //delete this->mmCatalog[topNode];
    this->mmCatalog.remove(topNode);
}

QString TreeCatalog::removeSubNode(const QString& parentNode, const int index)
{
    QString name = this->getSubNodeName(parentNode, index);
    if (index < this->mmCatalog[parentNode]->size()) this->mmCatalog[parentNode]->removeAt(index);
    return name;
}

QString TreeCatalog::getSubNodeName(const QString& parentNode, const int index)
{
    if (index < this->mmCatalog[parentNode]->size())
    {
        return this->mmCatalog[parentNode]->at(index);
    }
    return QString("null");
}

QList<QString> TreeCatalog::keys()
{
    return this->mmCatalog.keys();
}

QList<QList<QString>*> TreeCatalog::values()
{
    return this->mmCatalog.values();
}

int TreeCatalog::getKeyIndex(const QString& parentNode)
{
    int counter = 0;
    for (QString key: this->mmCatalog.keys())
    {
        if (key.compare(parentNode) == 0) return counter;
        ++counter;
    }
    return 0;
}

void TreeCatalog::clear()
{
    QMap<QString, QList<QString>*>::iterator it;
    for (it = this->mmCatalog.begin();
         it != this->mmCatalog.end();
         ++it)
    {
        //std::cout << it.key().toStdString() << std::endl;
        //std::cout << (*it.value())[0].toStdString() << std::endl;
        // is error: delete[] it.value();
        delete it.value();
        this->mmCatalog[it.key()] = nullptr;
    }
    this->mmCatalog.clear();
}

//void TreeCatalog::pushBack(const QString topNode)
//{
//    this->mmCatalog[topNode] = nullptr;
//}

TreeCatalog::~TreeCatalog()
{
    this->clear();
}
