#include <map>
#include <vector>
#include <utility>
#include <string>
#include "umb.h"
#define MAX_LEN 900 * 900 * 2
class UmbStandard{
public:
    UmbStandard(std::map<umb_cb_recv, std::pair<int, int>> sub_list,
        std::map<std::string, std::pair<int, int>> pub_list);
    ~UmbStandard();
    bool init_done = false;
    void send_umb_messeage(umb_inst_id_t inst_id, void *msg, unsigned len);
    std::map<std::string, umb_inst_id_t> pub_map;
private:
    //std::map<std::string, umb_inst_id_t> pub_map;
};

static void umb_initdone(void *param)
{
    (void)param;
}

static int cb_recv(umb_inst_id_t inst_id_in, umb_msgbuff_t *data, umb_timestamp_t *tm, void *param){
    return 0;
}

UmbStandard::UmbStandard(std::map<umb_cb_recv, std::pair<int, int>> sub_list,
    std::map<std::string, std::pair<int, int>> pub_list){
    std::map<umb_cb_recv, std::pair<int, int>>::iterator it;
    std::map<std::string, std::pair<int, int>>::iterator it_pub;
    pub_map.clear();
    //umb_errcode_e umb_ret = UMB_ERRCODE_OK;
    umb_init(umb_initdone, NULL);
    it = sub_list.begin();
    int counter = 0;
    umb_inst_id_t *inst_id_recv;
    while(it != sub_list.end())
    {

        inst_id_recv = new umb_inst_id_t;
        *inst_id_recv = counter;
        umb_sub_reg(inst_id_recv,
                    (umb_domain_id_t)(it->second).first,
                    (umb_topic_id_t)(it->second).second, it->first, NULL);
        it ++;
        counter++;
    }
    it_pub = pub_list.begin();
    while(it_pub != pub_list.end())
    {
        inst_id_recv = new umb_inst_id_t;
        *inst_id_recv = counter;
        umb_pub_reg(inst_id_recv,
                    (umb_domain_id_t)(it_pub->second).first,
                    (umb_topic_id_t)(it_pub->second).second);
        pub_map.insert(std::make_pair(it_pub->first, *inst_id_recv));
        it_pub ++;
        counter++;
    }
    init_done = true;
}

void UmbStandard::send_umb_messeage(umb_inst_id_t inst_id, void *msg, unsigned len){
    umb_msgbuff_t data;
    umb_errcode_e umb_ret = UMB_ERRCODE_OK;

    data.buff = msg;
    data.len = len;

    umb_ret = umb_pub(inst_id, &data);
    if (umb_ret != UMB_ERRCODE_OK) {
        fprintf(stderr, "! Write error %d\n", umb_ret);
    }
    //printf("send data len:%u\n",len);
    return;
}
