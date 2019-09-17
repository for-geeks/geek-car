#include <map>
#include <vector>
#include <utility>
#include <string>
#include "umb.h"
#include "umb_standard.h"
#define MAX_LEN 900 * 900 * 2

static void umb_initdone(void *param)
{
    (void)param;
}
UmbStandard::~UmbStandard(void){
    //umb_uninit();

    for (unsigned int i = 0; i < inst_list.size(); i++){
        umb_unreg(*inst_list.at(i));
    }
}
UmbStandard::UmbStandard(std::map<umb_cb_recv, std::pair<int, int>> sub_list,
    std::map<std::string, std::pair<int, int>> pub_list){
    std::map<umb_cb_recv, std::pair<int, int>>::iterator it;
    std::map<std::string, std::pair<int, int>>::iterator it_pub;
    umb_init(umb_initdone, nullptr);
    pub_map.clear();
    //umb_errcode_e umb_ret = UMB_ERRCODE_OK;
    //
    it = sub_list.begin();
    int counter = 0;
    umb_inst_id_t *inst_id_recv;
    umb_qosparam_t qos;
    qos.frag = 1;
    qos.reliable = 0;
    qos.q_len = 0;
    while(it != sub_list.end())
    {

        inst_id_recv = new umb_inst_id_t;
        *inst_id_recv = counter;
        umb_callback_t callback;
        callback.cb_evt = nullptr;
        callback.cb_recv = it->first;
        callback.cb_param = nullptr;
        inst_list.push_back(inst_id_recv);
        umb_sub_adv_reg(inst_id_recv,
                    (umb_domain_id_t)(it->second).first,
                    (umb_topic_id_t)(it->second).second, &callback, &qos);
        it ++;
        counter++;
    }
    it_pub = pub_list.begin();
    while(it_pub != pub_list.end())
    {
        inst_id_recv = new umb_inst_id_t;
        *inst_id_recv = counter;
        inst_list.push_back(inst_id_recv);
        umb_pub_adv_reg(inst_id_recv,
                        (umb_domain_id_t)(it_pub->second).first,
                        (umb_topic_id_t)(it_pub->second).second, nullptr, &qos);
        /*
        umb_pub_reg(inst_id_recv,
                    (umb_domain_id_t)(it_pub->second).first,
                    (umb_topic_id_t)(it_pub->second).second);
                    */
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
        //fprintf(stderr, "! Write error %d\n", umb_ret);
    }
    //printf("send data len:%u\n",len);
    return;
}

