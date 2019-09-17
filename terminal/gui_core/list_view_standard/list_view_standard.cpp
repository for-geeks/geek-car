#include <map>
#include <vector>
#include <utility>
#include <string>
#include <QTreeWidgetItem>
#include <QTreeWidget>
#include "list_view_standard.h"
void ListViewStandard::display_item(std::string item_name, std::string signal_name, double value){
    if (name_item_map.count(item_name)){
        if (name_item_map[item_name].name_witem_map.count(signal_name) == 0){
            QTreeWidgetItem * item;
            item = new QTreeWidgetItem(name_item_map[item_name].origin_item,
                        QStringList(QString::fromStdString(signal_name + ":" + std::to_string(value))));
            item->setExpanded(true);
            name_item_map[item_name].name_witem_map.insert(std::make_pair(signal_name, item));
            name_item_map[item_name].origin_item->addChild(item);
            //name_item_map.insert(std::make_pair(item_name, *item_standard));
        }else{
            name_item_map[item_name].name_witem_map[signal_name]->setText(0,
                QString::fromStdString(signal_name + ":" + std::to_string(value)));
        }
    }else{
        ListItemStandard *item_standard;
        item_standard = new ListItemStandard();
        item_standard->origin_item = new QTreeWidgetItem(view,
            QStringList(QString::fromStdString(item_name)));
        item_standard->origin_item ->setExpanded(true);
        QTreeWidgetItem * item;
        item = new QTreeWidgetItem(item_standard->origin_item,
                    QStringList(QString::fromStdString(signal_name + ":" + std::to_string(value))));
        item->setExpanded(true);
        item_standard->name_witem_map.insert(std::make_pair(signal_name, item));
        item_standard->origin_item->addChild(item);
        name_item_map.insert(std::make_pair(item_name, *item_standard));
     }
}




