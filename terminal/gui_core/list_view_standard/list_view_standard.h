#ifndef LIST_VIEW_STANDARD_H
#define LIST_VIEW_STANDARD_H
#include <map>
#include <vector>
#include <utility>
#include <string>
#include <QtWidgets/QWidget>
#include <QListView>
#include <QTreeWidget>

class ListItemStandard{
public:
    ListItemStandard(){
        name_witem_map.clear();
    }
    ~ListItemStandard(){}
    std::map<std::string, QTreeWidgetItem*> name_witem_map;
    QTreeWidgetItem* origin_item;
private:

};

class ListViewStandard{
public:
    ListViewStandard(QTreeWidget* input){
        view = input;
        view->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
        view->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
        name_item_map.clear();
    }
    ~ListViewStandard(){

    }
    void display_item(std::string item_name, std::string signal_name, double value);
    std::map<std::string, ListItemStandard> name_item_map;
    QTreeWidget *view;
private:


};
#endif // UMB_STANDARD_H
