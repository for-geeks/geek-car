#ifndef UMB_STANDARD_H
#define UMB_STANDARD_H
#include <map>
#include <string>
#include <utility>
#include <vector>
#include "umb.h"

class UmbStandard {
 public:
  UmbStandard(std::map<umb_cb_recv, std::pair<int, int>> sub_list,
              std::map<std::string, std::pair<int, int>> pub_list);
  ~UmbStandard();
  bool init_done = false;
  void send_umb_messeage(umb_inst_id_t inst_id, void *msg, unsigned len);
  std::map<std::string, umb_inst_id_t> pub_map;
  std::vector<umb_inst_id_t *> inst_list;

 private:
  // std::map<std::string, umb_inst_id_t> pub_map;
};
#endif  // UMB_STANDARD_H
