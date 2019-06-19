#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"

using apollo::cyber::Time;
using apollo::cyber::message::RawMessage;

uint64_t count = 0;
uint64_t total_latency = 0;

void MessageCallback(const std::shared_ptr<RawMessage>& msg) {
  uint64_t now = Time::Now().ToNanosecond();
  std::string ts(msg->message, 307200);
  total_latency += now - std::stoul(ts);
  ++count;
}

int main(int argc, char* argv[]) {
  // init cyber framework
  apollo::cyber::Init(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_v = 3;
  google::ParseCommandLineFlags(&argc, &argv, true);
  // create listener node
  auto listener_node = apollo::cyber::CreateNode("listener");
  // create listener
  auto listener = listener_node->CreateReader<RawMessage>("channel/chatter",
                                                          MessageCallback);
  apollo::cyber::WaitForShutdown();
  std::cout << "count: " << count
            << " avg_latency_us: " << total_latency / count / 1000 << std::endl;
  return 0;
}