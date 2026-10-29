/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#undef PGO_TRAINING
#define PATH_TO_PGO_CONFIG "path_to_pgo_config"

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/error-model.h"
#include "ns3/global-route-manager.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/packet.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/qbb-helper.h"
#include <fstream>
#include <iostream>
#include <ns3/rdma-client-helper.h>
#include <ns3/rdma-client.h>
#include <ns3/rdma-driver.h>
#include <ns3/rdma.h>
#include <ns3/rdma-cc-trace.h>
#include <ns3/sim-setting.h>
#include <ns3/switch-node.h>
#include <time.h>
#include <map>
#include <sstream>
#include <unordered_map>
#include <vector>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("GENERIC_SIMULATION");

uint32_t cc_mode = 1;
bool enable_qcn = true, use_dynamic_pfc_threshold = true;
uint32_t packet_payload_size = 1000, l2_chunk_size = 0, l2_ack_interval = 0;
double pause_time = 5, simulator_stop_time = 3.01;
std::string topology_file, flow_file, trace_file, trace_output_file;
std::string fct_output_file = "fct.txt";
std::string pfc_output_file = "pfc.txt";

double alpha_resume_interval = 55, rp_timer, ewma_gain = 1 / 16;
double rate_decrease_interval = 4;
uint32_t fast_recovery_times = 5;
std::string rate_ai, rate_hai, min_rate = "100Mb/s";
std::string dctcp_rate_ai = "1000Mb/s";

bool clamp_target_rate = false, l2_back_to_zero = false;
double error_rate_per_link = 0.0;
uint32_t has_win = 1;
uint32_t global_t = 1;
uint32_t mi_thresh = 5;
bool var_win = false, fast_react = true;
bool multi_rate = true;
bool sample_feedback = false;
double pint_log_base = 1.05;
double pint_prob = 1.0;
double u_target = 0.95;
uint32_t int_multi = 1;
bool rate_bound = true;
int nic_total_pause_time =
    0; // slightly less than finish time without inefficiency in us

uint32_t ack_high_prio = 0;
uint64_t link_down_time = 0;
uint32_t link_down_A = 0, link_down_B = 0;

uint32_t enable_trace = 1;

uint32_t buffer_size = 16;

uint32_t qlen_dump_interval = 1000, qlen_mon_interval = 100;
uint64_t qlen_mon_start = 0, qlen_mon_end = 2100000000;
int headroom_factor = 3;
string qlen_mon_file;

unordered_map<uint64_t, uint32_t> rate2kmax, rate2kmin;
unordered_map<uint64_t, double> rate2pmax;

/************************************************
 * Research extensions. Every default below reproduces the behaviour of the
 * backend before these keys existed. See
 * cca-ccl-study/docs/network-backend-extensions.md.
 ***********************************************/
// TRANSPORT_CONNECTION_MODE LEGACY|PERSISTENT
//   LEGACY: one sim_send -> one source port -> one QP -> fresh CC state.
//   PERSISTENT: one QP per (src, dst, priority, channel), reused for every
//   logical message; CC state and timers survive between messages.
std::string transport_connection_mode = "LEGACY";
// ENABLE_PFC 0|1. 0 stops switches from generating PAUSE frames, makes NICs
// and switches ignore received PAUSE frames, and sets headroom to 0.
uint32_t enable_pfc = 1;
// PFC_THRESHOLD_VALIDATION LEGACY|STRICT. STRICT aborts when the summed
// headroom and reserve of a switch reach its buffer (the subtraction in
// SwitchMmu::GetPfcThreshold is unsigned and would wrap). LEGACY only warns.
std::string pfc_threshold_validation = "LEGACY";
// QUEUE_LOG_MIN_BYTES: ports below this occupancy are left out of qlen.txt.
uint32_t queue_log_min_bytes = 1000;
// SWITCH_BUFFER_MAP_FILE: optional "node_id buffer_MiB" lines overriding
// BUFFER_SIZE for the listed switches only.
std::string switch_buffer_map_file;
std::map<uint32_t, uint32_t> switch_buffer_overrides;
// ENABLE_CCA_TRACE 0|1|2 and CCA_TRACE_FILE (see rdma-cc-trace.h).
uint32_t enable_cca_trace = 0;
std::string cca_trace_file = "cca_trace.csv";
// ENABLE_FLOW_METADATA_TRACE 0|1 and FLOW_TRACE_FILE: one CSV row per
// logical message (written by the frontend).
uint32_t enable_flow_metadata_trace = 0;
std::string flow_trace_file = "flows.csv";
// ENABLE_PATH_TRACE 0|1: record the ECMP path of every connection/flow in
// the flow trace and in PATH_TRACE_FILE.
uint32_t enable_path_trace = 0;
std::string path_trace_file = "paths.csv";
// TRACE_LEVEL 0|1|2. 2 keeps the packet trace (mix.tr, subject to
// ENABLE_TRACE) and the queue monitor. 1 drops the packet trace. 0 drops
// both. Counters, fct.txt, pfc.txt and the optional traces above are not
// affected.
uint32_t trace_level = 2;
// COUNTERS_OUTPUT_FILE: backend counters written at exit (empty = stdout only).
std::string counters_output_file;
// STATE_SNAPSHOT_TIMES n t1 ... tn (ns) and SNAPSHOT_OUTPUT_FILE.
std::vector<uint64_t> state_snapshot_times;
std::string snapshot_output_file = "snapshots.txt";
// LINK_REPORT_FILE: per-link, per-port and per-pair BDP/headroom/buffer
// values as the backend computed them (empty = not written).
std::string link_report_file;

/************************************************
 * Runtime varibles
 ***********************************************/
std::ifstream topof, flowf, tracef;

NodeContainer n;

uint64_t nic_rate;

uint64_t maxRtt, maxBdp;

std::vector<Ipv4Address> serverAddress;

// maintain port number for each host pair
std::unordered_map<uint32_t, unordered_map<uint32_t, uint16_t>> portNumber;

struct Interface {
  uint32_t idx;
  bool up;
  uint64_t delay;
  uint64_t bw;

  Interface() : idx(0), up(false) {}
};
map<Ptr<Node>, map<Ptr<Node>, Interface>> nbr2if;
// Mapping destination to next hop for each node: <node, <dest, <nexthop0, ...>
// > >
map<Ptr<Node>, map<Ptr<Node>, vector<Ptr<Node>>>> nextHop;
map<Ptr<Node>, map<Ptr<Node>, uint64_t>> pairDelay;
map<Ptr<Node>, map<Ptr<Node>, uint64_t>> pairTxDelay;
map<uint32_t, map<uint32_t, uint64_t>> pairBw;
map<Ptr<Node>, map<Ptr<Node>, uint64_t>> pairBdp;
map<uint32_t, map<uint32_t, uint64_t>> pairRtt;

struct FlowInput {
  uint32_t src, dst, pg, maxPacketCount, port, dport;
  double start_time;
  uint32_t idx;
};

FlowInput flow_input = {0};
uint32_t flow_num;
Ipv4Address node_id_to_ip(uint32_t id) {
  return Ipv4Address(0x0b000001 + ((id / 256) * 0x00010000) +
                     ((id % 256) * 0x00000100));
}

uint32_t ip_to_node_id(Ipv4Address ip) { return (ip.Get() >> 8) & 0xffff; }

void get_pfc(FILE *fout, Ptr<QbbNetDevice> dev, uint32_t type) {
  fprintf(fout, "%lu %u %u %u %u\n", Simulator::Now().GetTimeStep(),
          dev->GetNode()->GetId(), dev->GetNode()->GetNodeType(),
          dev->GetIfIndex(), type);
}

struct QlenDistribution {
  vector<uint32_t>
      cnt; // cnt[i] is the number of times that the queue len is i KB

  void add(uint32_t qlen) {
    uint32_t kb = qlen / 1000;
    if (cnt.size() < kb + 1)
      cnt.resize(kb + 1);
    cnt[kb]++;
  }
};
map<uint32_t, map<uint32_t, uint32_t>> queue_result;
void monitor_buffer(FILE *qlen_output, NodeContainer *n) {
  uint64_t now = Simulator::Now().GetTimeStep();
  // QLEN_MON_END is the last sample time. Without this check the monitor
  // reschedules itself forever, so Simulator::Run never drains even after
  // every rank has finished (and fflush of an empty NFS file every interval
  // keeps the process in iowait).
  if (now > qlen_mon_end)
    return;

  bool wrote = false;
  for (uint32_t i = 0; i < n->GetN(); i++) {
    if (n->Get(i)->GetNodeType() == 1) { // is switch
      Ptr<SwitchNode> sw = DynamicCast<SwitchNode>(n->Get(i));
      if (queue_result.find(i) == queue_result.end())
        queue_result[i];
      int test = 0;
      for (uint32_t j = 1; j < sw->GetNDevices(); j++) {
        uint32_t size = 0;
        for (uint32_t k = 0; k < SwitchMmu::qCnt; k++)
          size += sw->m_mmu->egress_bytes[j][k];
        if (size >= queue_log_min_bytes) {
          queue_result[i][j] = size;
          if (test == 0) {
            test = 1;
            fprintf(qlen_output, "time %lu %u ", now, i);
          }
          if (j < sw->GetNDevices() - 1) {
            test = 2;
            fprintf(qlen_output, "j %u %u ", j, size);
          } else if (j == sw->GetNDevices() - 1) {
            fprintf(qlen_output, "j %u %u\n", j, size);
            test = 3;
          }
        }
        if (j == sw->GetNDevices() - 1 && test == 2) {
          fprintf(qlen_output, "\n");
        }
      }
      if (test != 0)
        wrote = true;
    }
  }
  if (wrote)
    fflush(qlen_output);
  uint64_t next = now + qlen_mon_interval;
  if (next <= qlen_mon_end)
    Simulator::Schedule(NanoSeconds(qlen_mon_interval), &monitor_buffer,
                        qlen_output, n);
}

// Backend counters as "key=value" tokens on one line.
void write_counters_line(FILE *out, const char *prefix) {
  uint64_t active_qps = 0, pending_msgs = 0;
  for (uint32_t i = 0; i < n.GetN(); i++) {
    if (n.Get(i)->GetNodeType() != 0)
      continue;
    Ptr<RdmaDriver> rdma = n.Get(i)->GetObject<RdmaDriver>();
    if (!rdma || !rdma->m_rdma)
      continue;
    active_qps += rdma->m_rdma->m_qpMap.size();
    for (auto &kv : rdma->m_rdma->m_qpMap)
      pending_msgs += kv.second->m_persistent ? kv.second->m_wrs.size()
                                              : (kv.second->IsFinished() ? 0 : 1);
  }
  fprintf(out,
          "%s time_ns=%lu ecn_marks=%lu cnp_acks=%lu mmu_drops=%lu pfc_pause_frames=%lu "
          "pfc_resume_frames=%lu qps_created=%lu qps_destroyed=%lu active_qps=%lu "
          "messages_posted=%lu messages_completed=%lu messages_pending=%lu\n",
          prefix, (unsigned long)Simulator::Now().GetTimeStep(),
          (unsigned long)RdmaCounters::ecn_marks, (unsigned long)RdmaCounters::cnp_acks,
          (unsigned long)RdmaCounters::mmu_drops,
          (unsigned long)RdmaCounters::pfc_pause_frames,
          (unsigned long)RdmaCounters::pfc_resume_frames,
          (unsigned long)RdmaCounters::qps_created,
          (unsigned long)RdmaCounters::qps_destroyed, (unsigned long)active_qps,
          (unsigned long)RdmaCounters::messages_posted,
          (unsigned long)RdmaCounters::messages_completed, (unsigned long)pending_msgs);
}

// STATE_SNAPSHOT_TIMES: dump every live QP's controller state, every switch
// port's occupancy and pause state, and the cumulative counters. Appends to
// SNAPSHOT_OUTPUT_FILE. Observational only.
void write_state_snapshot(uint64_t requested_ns) {
  static FILE *out = NULL;
  if (!out) {
    out = fopen(snapshot_output_file.c_str(), "w");
    if (!out)
      return;
  }
  fprintf(out, "snapshot requested_ns=%lu time_ns=%lu\n", (unsigned long)requested_ns,
          (unsigned long)Simulator::Now().GetTimeStep());
  write_counters_line(out, "counters");
  for (uint32_t i = 0; i < n.GetN(); i++) {
    if (n.Get(i)->GetNodeType() != 0)
      continue;
    Ptr<RdmaDriver> rdma = n.Get(i)->GetObject<RdmaDriver>();
    if (!rdma || !rdma->m_rdma)
      continue;
    for (auto &kv : rdma->m_rdma->m_qpMap) {
      Ptr<RdmaQueuePair> qp = kv.second;
      uint64_t window = cc_mode == 3 ? qp->HpGetCurWin() : qp->GetWin();
      fprintf(out,
              "qp node=%u conn_id=%lu qp_id=%lu src=%u dst=%u sport=%u dport=%u persistent=%d "
              "rate_bps=%lu target_rate_bps=%lu alpha=%.9f hp_cur_rate_bps=%lu "
              "window_bytes=%lu bytes_left=%lu snd_nxt=%lu snd_una=%lu pending_msgs=%lu "
              "msgs_posted=%lu msgs_completed=%lu cnp_count=%lu rp_stage=%u "
              "first_cnp_pending=%d\n",
              i, (unsigned long)qp->m_conn_id, (unsigned long)qp->m_qp_id, qp->m_src,
              qp->m_dest, qp->sport, qp->dport, qp->m_persistent ? 1 : 0,
              (unsigned long)qp->m_rate.GetBitRate(),
              (unsigned long)qp->mlx.m_targetRate.GetBitRate(), qp->mlx.m_alpha,
              (unsigned long)qp->hp.m_curRate.GetBitRate(), (unsigned long)window,
              (unsigned long)qp->GetBytesLeft(), (unsigned long)qp->snd_nxt,
              (unsigned long)qp->snd_una, (unsigned long)qp->m_wrs.size(),
              (unsigned long)qp->m_messages_posted, (unsigned long)qp->m_messages_completed,
              (unsigned long)qp->m_cnp_count, qp->mlx.m_rpTimeStage,
              qp->mlx.m_first_cnp ? 1 : 0);
    }
  }
  for (uint32_t i = 0; i < n.GetN(); i++) {
    if (n.Get(i)->GetNodeType() != 1)
      continue;
    Ptr<SwitchNode> sw = DynamicCast<SwitchNode>(n.Get(i));
    for (uint32_t j = 1; j < sw->GetNDevices(); j++) {
      uint64_t bytes = 0;
      uint32_t paused_mask = 0;
      for (uint32_t k = 0; k < SwitchMmu::qCnt; k++) {
        bytes += sw->m_mmu->egress_bytes[j][k];
        if (sw->m_mmu->paused[j][k])
          paused_mask |= (1u << k);
      }
      fprintf(out,
              "port node=%u ifindex=%u egress_bytes=%lu ingress_paused_mask=%u "
              "tx_bytes=%lu ecn_marks=%lu mmu_drops=%lu pfc_pause_tx=%lu\n",
              i, j, (unsigned long)bytes, paused_mask, (unsigned long)sw->GetTxBytes(j),
              (unsigned long)sw->m_ecnMarks[j], (unsigned long)sw->m_mmuDrops[j],
              (unsigned long)sw->m_pfcPauseTx[j]);
    }
  }
  fprintf(out, "end\n");
  fflush(out);
}

void CalculateRoute(Ptr<Node> host) {
  // queue for the BFS.
  vector<Ptr<Node>> q;
  // Distance from the host to each node.
  map<Ptr<Node>, int> dis;
  map<Ptr<Node>, uint64_t> delay;
  map<Ptr<Node>, uint64_t> txDelay;
  map<Ptr<Node>, uint64_t> bw;
  // init BFS.
  q.push_back(host);
  dis[host] = 0;
  delay[host] = 0;
  txDelay[host] = 0;
  bw[host] = 0xfffffffffffffffflu;
  // BFS.
  for (int i = 0; i < (int)q.size(); i++) {
    Ptr<Node> now = q[i];
    int d = dis[now];
    for (auto it = nbr2if[now].begin(); it != nbr2if[now].end(); it++) {
      // skip down link
      if (!it->second.up)
        continue;
      Ptr<Node> next = it->first;
      if (dis.find(next) == dis.end()) {
        dis[next] = d + 1;
        delay[next] = delay[now] + it->second.delay;
        txDelay[next] = txDelay[now] +
                        packet_payload_size * 1000000000lu * 8 / it->second.bw;
        bw[next] = std::min(bw[now], it->second.bw);
        if (next->GetNodeType() == 1)
          q.push_back(next);
      }
      if (d + 1 == dis[next]) {
        nextHop[next][host].push_back(now);
      }
    }
  }
  for (auto it : delay) {
    // std::cout << "pairDelay first "<< it.first->GetId() << " host " <<
    // host->GetId() << " delay " << it.second << std::endl;
    pairDelay[it.first][host] = it.second;
  }
  for (auto it : txDelay)
    pairTxDelay[it.first][host] = it.second;
  for (auto it : bw) {
    // std::cout << "pairBw first "<< it.first->GetId() << " host " <<
    // host->GetId() << " bw " << it.second << std::endl;
    pairBw[it.first->GetId()][host->GetId()] = it.second;
  }
}

void CalculateRoutes(NodeContainer &n) {
  for (int i = 0; i < (int)n.GetN(); i++) {
    Ptr<Node> node = n.Get(i);
    if (node->GetNodeType() == 0)
      CalculateRoute(node);
  }
}

void SetRoutingEntries() {
  for (auto i = nextHop.begin(); i != nextHop.end(); i++) {
    Ptr<Node> node = i->first;
    auto &table = i->second;
    for (auto j = table.begin(); j != table.end(); j++) {
      Ptr<Node> dst = j->first;
      Ipv4Address dstAddr = dst->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
      vector<Ptr<Node>> nexts = j->second;
      for (int k = 0; k < (int)nexts.size(); k++) {
        Ptr<Node> next = nexts[k];
        uint32_t interface = nbr2if[node][next].idx;
        if (node->GetNodeType() == 1)
          DynamicCast<SwitchNode>(node)->AddTableEntry(dstAddr, interface);
        else {
          node->GetObject<RdmaDriver>()->m_rdma->AddTableEntry(dstAddr,
                                                               interface);
        }
      }
    }
  }
}

// take down the link between a and b, and redo the routing
void TakeDownLink(NodeContainer n, Ptr<Node> a, Ptr<Node> b) {
  if (!nbr2if[a][b].up)
    return;
  // take down link between a and b
  nbr2if[a][b].up = nbr2if[b][a].up = false;
  nextHop.clear();
  CalculateRoutes(n);
  // clear routing tables
  for (uint32_t i = 0; i < n.GetN(); i++) {
    if (n.Get(i)->GetNodeType() == 1)
      DynamicCast<SwitchNode>(n.Get(i))->ClearTable();
    else
      n.Get(i)->GetObject<RdmaDriver>()->m_rdma->ClearTable();
  }
  DynamicCast<QbbNetDevice>(a->GetDevice(nbr2if[a][b].idx))->TakeDown();
  DynamicCast<QbbNetDevice>(b->GetDevice(nbr2if[b][a].idx))->TakeDown();
  // reset routing table
  SetRoutingEntries();

  // redistribute qp on each host
  for (uint32_t i = 0; i < n.GetN(); i++) {
    if (n.Get(i)->GetNodeType() == 0)
      n.Get(i)->GetObject<RdmaDriver>()->m_rdma->RedistributeQp();
  }
}

uint64_t get_nic_rate(NodeContainer &n) {
  for (uint32_t i = 0; i < n.GetN(); i++)
    if (n.Get(i)->GetNodeType() == 0)
      return DynamicCast<QbbNetDevice>(n.Get(i)->GetDevice(1))
          ->GetDataRate()
          .GetBitRate();
  // TODO: Complain if you cannot find the NIC rate.
  return 0;
}

bool ReadConf(string network_configuration) {
  // Read the configuration file
  std::ifstream conf;
  conf.open(network_configuration);
  if (!conf.is_open()) {
    std::cout << "Error: cannot find network config file: " << network_configuration << std::endl;
    fflush(stdout);
    return false;
  }
  
  while (!conf.eof()) {
    std::string key;
    conf >> key;

    if (key.compare("ENABLE_QCN") == 0) {
      uint32_t v;
      conf >> v;
      enable_qcn = v;
    } else if (key.compare("USE_DYNAMIC_PFC_THRESHOLD") == 0) {
      uint32_t v;
      conf >> v;
      use_dynamic_pfc_threshold = v;
    } else if (key.compare("CLAMP_TARGET_RATE") == 0) {
      uint32_t v;
      conf >> v;
      clamp_target_rate = v;
    } else if (key.compare("PAUSE_TIME") == 0) {
      double v;
      conf >> v;
      pause_time = v;
    } else if (key.compare("PACKET_PAYLOAD_SIZE") == 0) {
      uint32_t v;
      conf >> v;
      packet_payload_size = v;
    } else if (key.compare("L2_CHUNK_SIZE") == 0) {
      uint32_t v;
      conf >> v;
      l2_chunk_size = v;
    } else if (key.compare("L2_ACK_INTERVAL") == 0) {
      uint32_t v;
      conf >> v;
      l2_ack_interval = v;
    } else if (key.compare("L2_BACK_TO_ZERO") == 0) {
      uint32_t v;
      conf >> v;
      l2_back_to_zero = v;
    } else if (key.compare("TOPOLOGY_FILE") == 0) {
      std::string v;
      conf >> v;
      topology_file = v;
    } else if (key.compare("FLOW_FILE") == 0) {
      std::string v;
      conf >> v;
      flow_file = v;
    } else if (key.compare("TRACE_FILE") == 0) {
      std::string v;
      conf >> v;
      trace_file = v;
    } else if (key.compare("TRACE_OUTPUT_FILE") == 0) {
      std::string v;
      conf >> v;
      trace_output_file = v;
      // Removed to handle new command line arguments in build.sh.
  //if (argc > 2) {
      //  trace_output_file = trace_output_file + std::string(argv[2]);
      //}
    } else if (key.compare("SIMULATOR_STOP_TIME") == 0) {
      double v;
      conf >> v;
      simulator_stop_time = v;
    } else if (key.compare("ALPHA_RESUME_INTERVAL") == 0) {
      double v;
      conf >> v;
      alpha_resume_interval = v;
    } else if (key.compare("RP_TIMER") == 0) {
      double v;
      conf >> v;
      rp_timer = v;
    } else if (key.compare("EWMA_GAIN") == 0) {
      double v;
      conf >> v;
      ewma_gain = v;
    } else if (key.compare("FAST_RECOVERY_TIMES") == 0) {
      uint32_t v;
      conf >> v;
      fast_recovery_times = v;
    } else if (key.compare("RATE_AI") == 0) {
      std::string v;
      conf >> v;
      rate_ai = v;
    } else if (key.compare("RATE_HAI") == 0) {
      std::string v;
      conf >> v;
      rate_hai = v;
    } else if (key.compare("ERROR_RATE_PER_LINK") == 0) {
      double v;
      conf >> v;
      error_rate_per_link = v;
    } else if (key.compare("CC_MODE") == 0) {
      conf >> cc_mode;
    } else if (key.compare("RATE_DECREASE_INTERVAL") == 0) {
      double v;
      conf >> v;
      rate_decrease_interval = v;
    } else if (key.compare("MIN_RATE") == 0) {
      conf >> min_rate;
    } else if (key.compare("FCT_OUTPUT_FILE") == 0) {
      conf >> fct_output_file;
    } else if (key.compare("HAS_WIN") == 0) {
      conf >> has_win;
    } else if (key.compare("GLOBAL_T") == 0) {
      conf >> global_t;
    } else if (key.compare("MI_THRESH") == 0) {
      conf >> mi_thresh;
    } else if (key.compare("VAR_WIN") == 0) {
      uint32_t v;
      conf >> v;
      var_win = v;
    } else if (key.compare("FAST_REACT") == 0) {
      uint32_t v;
      conf >> v;
      fast_react = v;
    } else if (key.compare("U_TARGET") == 0) {
      conf >> u_target;
    } else if (key.compare("INT_MULTI") == 0) {
      conf >> int_multi;
    } else if (key.compare("RATE_BOUND") == 0) {
      uint32_t v;
      conf >> v;
      rate_bound = v;
    } else if (key.compare("ACK_HIGH_PRIO") == 0) {
      conf >> ack_high_prio;
    } else if (key.compare("DCTCP_RATE_AI") == 0) {
      conf >> dctcp_rate_ai;
    } else if (key.compare("NIC_TOTAL_PAUSE_TIME") == 0) {
      conf >> nic_total_pause_time;
    } else if (key.compare("PFC_OUTPUT_FILE") == 0) {
      conf >> pfc_output_file;
    } else if (key.compare("LINK_DOWN") == 0) {
      conf >> link_down_time >> link_down_A >> link_down_B;
    } else if (key.compare("ENABLE_TRACE") == 0) {
      conf >> enable_trace;
    } else if (key.compare("KMAX_MAP") == 0) {
      int n_k;
      conf >> n_k;
      for (int i = 0; i < n_k; i++) {
        uint64_t rate;
        uint32_t k;
        conf >> rate >> k;
        rate2kmax[rate] = k;
      }
    } else if (key.compare("KMIN_MAP") == 0) {
      int n_k;
      conf >> n_k;
      for (int i = 0; i < n_k; i++) {
        uint64_t rate;
        uint32_t k;
        conf >> rate >> k;
        rate2kmin[rate] = k;
      }
    } else if (key.compare("PMAX_MAP") == 0) {
      int n_k;
      conf >> n_k;
      for (int i = 0; i < n_k; i++) {
        uint64_t rate;
        double p;
        conf >> rate >> p;
        rate2pmax[rate] = p;
      }
    } else if (key.compare("BUFFER_SIZE") == 0) {
      conf >> buffer_size;
    } else if (key.compare("QLEN_MON_FILE") == 0) {
      conf >> qlen_mon_file;
    } else if (key.compare("QLEN_MON_START") == 0) {
      conf >> qlen_mon_start;
    } else if (key.compare("QLEN_MON_END") == 0) {
      conf >> qlen_mon_end;
    } else if (key.compare("QLEN_MON_INTERVAL") == 0) {
      // sampling period of monitor_buffer in ns (default 100, as before)
      conf >> qlen_mon_interval;
    } else if (key.compare("MULTI_RATE") == 0) {
      int v;
      conf >> v;
      multi_rate = v;
    } else if (key.compare("SAMPLE_FEEDBACK") == 0) {
      int v;
      conf >> v;
      sample_feedback = v;
    } else if (key.compare("PINT_LOG_BASE") == 0) {
      conf >> pint_log_base;
    } else if (key.compare("PINT_PROB") == 0) {
      conf >> pint_prob;
    } else if (key == "HEADROOM_FACTOR") {
      int v;
      conf >> v;
      headroom_factor = v;
      std::cout << "headroom factor set to: " << headroom_factor << std::endl;
    } else if (key == "TRANSPORT_CONNECTION_MODE") {
      conf >> transport_connection_mode;
      if (transport_connection_mode != "LEGACY" &&
          transport_connection_mode != "PERSISTENT") {
        std::cerr << "TRANSPORT_CONNECTION_MODE must be LEGACY or PERSISTENT, got "
                  << transport_connection_mode << std::endl;
        return false;
      }
    } else if (key == "ENABLE_PFC") {
      conf >> enable_pfc;
    } else if (key == "PFC_THRESHOLD_VALIDATION") {
      conf >> pfc_threshold_validation;
      if (pfc_threshold_validation != "LEGACY" &&
          pfc_threshold_validation != "STRICT") {
        std::cerr << "PFC_THRESHOLD_VALIDATION must be LEGACY or STRICT, got "
                  << pfc_threshold_validation << std::endl;
        return false;
      }
    } else if (key == "QUEUE_LOG_MIN_BYTES") {
      conf >> queue_log_min_bytes;
    } else if (key == "SWITCH_BUFFER_MAP_FILE") {
      conf >> switch_buffer_map_file;
    } else if (key == "ENABLE_CCA_TRACE") {
      conf >> enable_cca_trace;
    } else if (key == "CCA_TRACE_FILE") {
      conf >> cca_trace_file;
    } else if (key == "ENABLE_FLOW_METADATA_TRACE") {
      conf >> enable_flow_metadata_trace;
    } else if (key == "FLOW_TRACE_FILE") {
      conf >> flow_trace_file;
    } else if (key == "ENABLE_PATH_TRACE") {
      conf >> enable_path_trace;
    } else if (key == "PATH_TRACE_FILE") {
      conf >> path_trace_file;
    } else if (key == "TRACE_LEVEL") {
      conf >> trace_level;
    } else if (key == "COUNTERS_OUTPUT_FILE") {
      conf >> counters_output_file;
    } else if (key == "STATE_SNAPSHOT_TIMES") {
      int n_t;
      conf >> n_t;
      for (int i = 0; i < n_t; i++) {
        uint64_t t;
        conf >> t;
        state_snapshot_times.push_back(t);
      }
    } else if (key == "SNAPSHOT_OUTPUT_FILE") {
      conf >> snapshot_output_file;
    } else if (key == "LINK_REPORT_FILE") {
      conf >> link_report_file;
    }
    fflush(stdout);
  }
  conf.close();
  return true;
}

// SWITCH_BUFFER_MAP_FILE: "node_id buffer_MiB" per line, '#' comments.
// Validation against the topology happens in SetupNetwork once nodes exist.
bool ReadSwitchBufferMap() {
  if (switch_buffer_map_file.empty())
    return true;
  std::ifstream f(switch_buffer_map_file.c_str());
  if (!f.is_open()) {
    std::cerr << "Error: cannot open SWITCH_BUFFER_MAP_FILE " << switch_buffer_map_file
              << std::endl;
    return false;
  }
  std::string line;
  while (std::getline(f, line)) {
    size_t hash = line.find('#');
    if (hash != std::string::npos)
      line = line.substr(0, hash);
    std::istringstream iss(line);
    long long node, mib;
    if (!(iss >> node >> mib))
      continue;
    if (node < 0 || mib <= 0) {
      std::cerr << "SWITCH_BUFFER_MAP_FILE: invalid entry '" << line
                << "' (node >= 0 and buffer_MiB > 0 required)" << std::endl;
      return false;
    }
    switch_buffer_overrides[(uint32_t)node] = (uint32_t)mib;
  }
  return true;
}

void SetConfig() {
  bool dynamicth = use_dynamic_pfc_threshold;

  Config::SetDefault("ns3::QbbNetDevice::PauseTime", UintegerValue(pause_time));
  Config::SetDefault("ns3::QbbNetDevice::QcnEnabled", BooleanValue(enable_qcn));
  Config::SetDefault("ns3::QbbNetDevice::DynamicThreshold",
                     BooleanValue(dynamicth));
  if (!enable_pfc) {
    // Devices ignore received PAUSE frames; switches stop generating them
    // (SwitchNode::PfcEnabled, set per switch below).
    Config::SetDefault("ns3::QbbNetDevice::QbbEnabled", BooleanValue(false));
  }
  // TRACE_LEVEL 0/1 remove the packet trace; 0 also removes the queue monitor.
  if (trace_level < 2)
    enable_trace = 0;
  RdmaCcTrace::cc_mode = cc_mode;
  if (enable_cca_trace > 0)
    RdmaCcTrace::Open(cca_trace_file, (int)enable_cca_trace);

  // set int_multi
  IntHop::multi = int_multi;
  // IntHeader::mode
  if (cc_mode == 7) // timely, use ts
    IntHeader::mode = IntHeader::TS;
  else if (cc_mode == 3) // hpcc, use int
    IntHeader::mode = IntHeader::NORMAL;
  else if (cc_mode == 10) // hpcc-pint
    IntHeader::mode = IntHeader::PINT;
  else // others, no extra header
    IntHeader::mode = IntHeader::NONE;

  // Set Pint
  if (cc_mode == 10) {
    Pint::set_log_base(pint_log_base);
    IntHeader::pint_bytes = Pint::get_n_bytes();
    printf("PINT bits: %d bytes: %d\n", Pint::get_n_bits(),
           Pint::get_n_bytes());
  }
}

bool SetupNetwork(void (*qp_finish)(FILE *, Ptr<RdmaQueuePair>)) {

  topof.open(topology_file.c_str());
  if (!topof.is_open()) {
    std::cerr << "Error: cannot open topology file: " << topology_file << std::endl;
    return false;
  }

  flowf.open(flow_file.c_str());
  if (!flowf.is_open()) {
    std::cerr << "Error: cannot open flow file: " << flow_file << std::endl;
    return false;
  }

  tracef.open(trace_file.c_str());
  if (!tracef.is_open()) {
    std::cerr << "Error: cannot open trace file: " << trace_file << std::endl;
    return false;
  }

  uint32_t node_num, switch_num, link_num, trace_num;
  topof >> node_num >> switch_num >> link_num;
  flowf >> flow_num;
  tracef >> trace_num;

  std::vector<uint32_t> node_type(node_num, 0);
  for (uint32_t i = 0; i < switch_num; i++) {
    uint32_t sid;
    topof >> sid;
    node_type[sid] = 1;
  }
  for (uint32_t i = 0; i < node_num; i++) {
    if (node_type[i] == 0)
      n.Add(CreateObject<Node>());
    else {
      Ptr<SwitchNode> sw = CreateObject<SwitchNode>();
      n.Add(sw);
      sw->SetAttribute("EcnEnabled", BooleanValue(enable_qcn));
    }
  }

  NS_LOG_INFO("Create nodes.");

  InternetStackHelper internet;
  internet.Install(n);

  //
  // Assign IP to each server
  //
  for (uint32_t i = 0; i < node_num; i++) {
    if (n.Get(i)->GetNodeType() == 0) {
      serverAddress.resize(i + 1);
      serverAddress[i] = node_id_to_ip(i);
    }
  }

  NS_LOG_INFO("Create channels.");

  Ptr<RateErrorModel> rem = CreateObject<RateErrorModel>();
  Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
  rem->SetRandomVariable(uv);
  uv->SetStream(50);
  rem->SetAttribute("ErrorRate", DoubleValue(error_rate_per_link));
  rem->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));

  FILE *pfc_file = fopen(pfc_output_file.c_str(), "w");

  QbbHelper qbb;
  Ipv4AddressHelper ipv4;
  for (uint32_t i = 0; i < link_num; i++) {
    uint32_t src, dst;
    std::string data_rate, link_delay;
    double error_rate;
    topof >> src >> dst >> data_rate >> link_delay >> error_rate;
    Ptr<Node> snode = n.Get(src), dnode = n.Get(dst);

    qbb.SetDeviceAttribute("DataRate", StringValue(data_rate));
    qbb.SetChannelAttribute("Delay", StringValue(link_delay));

    if (error_rate > 0) {
      Ptr<RateErrorModel> rem = CreateObject<RateErrorModel>();
      Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
      rem->SetRandomVariable(uv);
      uv->SetStream(50);
      rem->SetAttribute("ErrorRate", DoubleValue(error_rate));
      rem->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
      qbb.SetDeviceAttribute("ReceiveErrorModel", PointerValue(rem));
    } else {
      qbb.SetDeviceAttribute("ReceiveErrorModel", PointerValue(rem));
    }

    fflush(stdout);

    // Assigne server IP
    // Note: this should be before the automatic assignment below
    // (ipv4.Assign(d)), because we want our IP to be the primary IP (first in
    // the IP address list), so that the global routing is based on our IP
    NetDeviceContainer d = qbb.Install(snode, dnode);
    if (snode->GetNodeType() == 0) {
      Ptr<Ipv4> ipv4 = snode->GetObject<Ipv4>();
      ipv4->AddInterface(d.Get(0));
      ipv4->AddAddress(
          1, Ipv4InterfaceAddress(serverAddress[src], Ipv4Mask(0xff000000)));

    }
    if (dnode->GetNodeType() == 0) {
      Ptr<Ipv4> ipv4 = dnode->GetObject<Ipv4>();
      ipv4->AddInterface(d.Get(1));
      ipv4->AddAddress(
          1, Ipv4InterfaceAddress(serverAddress[dst], Ipv4Mask(0xff000000)));
    }

    // used to create a graph of the topology
    nbr2if[snode][dnode].idx =
        DynamicCast<QbbNetDevice>(d.Get(0))->GetIfIndex();
    nbr2if[snode][dnode].up = true;
    nbr2if[snode][dnode].delay =
        DynamicCast<QbbChannel>(
            DynamicCast<QbbNetDevice>(d.Get(0))->GetChannel())
            ->GetDelay()
            .GetTimeStep();
    nbr2if[snode][dnode].bw =
        DynamicCast<QbbNetDevice>(d.Get(0))->GetDataRate().GetBitRate();
    nbr2if[dnode][snode].idx =
        DynamicCast<QbbNetDevice>(d.Get(1))->GetIfIndex();
    nbr2if[dnode][snode].up = true;
    nbr2if[dnode][snode].delay =
        DynamicCast<QbbChannel>(
            DynamicCast<QbbNetDevice>(d.Get(1))->GetChannel())
            ->GetDelay()
            .GetTimeStep();
    nbr2if[dnode][snode].bw =
        DynamicCast<QbbNetDevice>(d.Get(1))->GetDataRate().GetBitRate();

    // This is just to set up the connectivity between nodes. The IP addresses
    // are useless
    char ipstring[16];
    sprintf(ipstring, "10.%d.%d.0", i / 254 + 1, i % 254 + 1);
    ipv4.SetBase(ipstring, "255.255.255.0");
    ipv4.Assign(d);

    // setup PFC trace
    DynamicCast<QbbNetDevice>(d.Get(0))->TraceConnectWithoutContext(
        "QbbPfc", MakeBoundCallback(&get_pfc, pfc_file,
                                    DynamicCast<QbbNetDevice>(d.Get(0))));
    DynamicCast<QbbNetDevice>(d.Get(1))->TraceConnectWithoutContext(
        "QbbPfc", MakeBoundCallback(&get_pfc, pfc_file,
                                    DynamicCast<QbbNetDevice>(d.Get(1))));
  }

  nic_rate = get_nic_rate(n);

  // Per-switch buffer overrides: validate against the topology first.
  if (!ReadSwitchBufferMap())
    return false;
  for (auto &kv : switch_buffer_overrides) {
    if (kv.first >= node_num) {
      std::cerr << "SWITCH_BUFFER_MAP_FILE: node " << kv.first << " does not exist ("
                << node_num << " nodes)" << std::endl;
      return false;
    }
    if (n.Get(kv.first)->GetNodeType() != 1) {
      std::cerr << "SWITCH_BUFFER_MAP_FILE: node " << kv.first << " is not a switch"
                << std::endl;
      return false;
    }
  }

  FILE *link_report = NULL;
  if (!link_report_file.empty()) {
    link_report = fopen(link_report_file.c_str(), "w");
    if (link_report) {
      fprintf(link_report, "# time_unit ns ; rate_unit bit/s ; sizes bytes\n");
      fprintf(link_report, "config buffer_size_mib=%u headroom_factor=%d enable_pfc=%u "
              "pfc_threshold_validation=%s transport_connection_mode=%s nic_rate_bps=%lu\n",
              buffer_size, headroom_factor, enable_pfc, pfc_threshold_validation.c_str(),
              transport_connection_mode.c_str(), (unsigned long)nic_rate);
      for (auto &i : nbr2if) {
        for (auto &j : i.second) {
          uint64_t bdp = j.second.bw * j.second.delay / 8 / 1000000000;
          fprintf(link_report, "link src=%u dst=%u ifindex=%u rate_bps=%lu delay_ns=%lu "
                  "one_way_bdp_bytes=%lu\n",
                  i.first->GetId(), j.first->GetId(), j.second.idx,
                  (unsigned long)j.second.bw, (unsigned long)j.second.delay,
                  (unsigned long)bdp);
        }
      }
    }
  }

  // config switch
  bool pfc_config_invalid = false;
  for (uint32_t i = 0; i < node_num; i++) {
    if (n.Get(i)->GetNodeType() == 1) { // is switch
      Ptr<SwitchNode> sw = DynamicCast<SwitchNode>(n.Get(i));
      uint32_t shift = 3; // by default 1/8
      if (!enable_pfc)
        sw->SetAttribute("PfcEnabled", BooleanValue(false));

      uint32_t this_buffer_mib = buffer_size;
      auto ov = switch_buffer_overrides.find(i);
      if (ov != switch_buffer_overrides.end())
        this_buffer_mib = ov->second;
      uint64_t this_buffer_bytes = (uint64_t)this_buffer_mib * 1024 * 1024;
      if (this_buffer_bytes > 0xffffffffULL) {
        std::cerr << "switch " << i << ": buffer " << this_buffer_mib
                  << " MiB does not fit SwitchMmu's 32-bit byte counters" << std::endl;
        return false;
      }

      for (uint32_t j = 1; j < sw->GetNDevices(); j++) {
        Ptr<QbbNetDevice> dev = DynamicCast<QbbNetDevice>(sw->GetDevice(j));
        // set ecn
        uint64_t rate = dev->GetDataRate().GetBitRate();
        NS_ASSERT_MSG(rate2kmin.find(rate) != rate2kmin.end(),
                      "must set kmin for each link speed");
        NS_ASSERT_MSG(rate2kmax.find(rate) != rate2kmax.end(),
                      "must set kmax for each link speed");
        NS_ASSERT_MSG(rate2pmax.find(rate) != rate2pmax.end(),
                      "must set pmax for each link speed");
        sw->m_mmu->ConfigEcn(j, rate2kmin[rate], rate2kmax[rate],
                             rate2pmax[rate]);
        // set pfc
        uint64_t delay = DynamicCast<QbbChannel>(dev->GetChannel())
                             ->GetDelay()
                             .GetTimeStep();
        // uint32_t headroom = 150000; // rate * delay / 8 / 1000000000 * 3;
        uint64_t headroom64 = rate * delay / 8 / 1000000000 * headroom_factor;
        if (!enable_pfc)
          headroom64 = 0; // no PAUSE frames, so nothing lands in headroom
        if (headroom64 > 0xffffffffULL) {
          std::cerr << "switch " << i << " port " << j << ": headroom " << headroom64
                    << " bytes overflows uint32 (rate " << rate << " bit/s, delay "
                    << delay << " ns)" << std::endl;
          if (pfc_threshold_validation == "STRICT")
            return false;
        }
        uint32_t headroom = (uint32_t)headroom64;
        sw->m_mmu->ConfigHdrm(j, headroom);

        // set pfc alpha, proportional to link bw
        sw->m_mmu->pfc_a_shift[j] = shift;
        while (rate > nic_rate && sw->m_mmu->pfc_a_shift[j] > 0) {
          sw->m_mmu->pfc_a_shift[j]--;
          rate /= 2;
        }
        if (link_report) {
          uint64_t link_rate = dev->GetDataRate().GetBitRate();
          fprintf(link_report, "port node=%u ifindex=%u rate_bps=%lu delay_ns=%lu "
                  "headroom_bytes=%u kmin_bytes=%u kmax_bytes=%u pmax=%g pfc_a_shift=%u\n",
                  i, j, (unsigned long)link_rate, (unsigned long)delay, headroom,
                  rate2kmin[link_rate] * 1000, rate2kmax[link_rate] * 1000,
                  rate2pmax[link_rate], sw->m_mmu->pfc_a_shift[j]);
        }
      }
      sw->m_mmu->ConfigNPort(sw->GetNDevices() - 1);
      sw->m_mmu->ConfigBufferSize((uint32_t)this_buffer_bytes);
      sw->m_mmu->node_id = sw->GetId();

      // Headroom + reserve must leave shared buffer, or
      // SwitchMmu::GetPfcThreshold wraps its unsigned subtraction.
      uint64_t committed = (uint64_t)sw->m_mmu->total_hdrm + sw->m_mmu->total_rsrv;
      bool invalid = committed >= this_buffer_bytes;
      if (link_report) {
        fprintf(link_report, "switch node=%u buffer_bytes=%lu total_headroom_bytes=%u "
                "total_reserve_bytes=%u shared_bytes=%ld pfc_config_valid=%d\n",
                i, (unsigned long)this_buffer_bytes, sw->m_mmu->total_hdrm,
                sw->m_mmu->total_rsrv,
                (long)((int64_t)this_buffer_bytes - (int64_t)committed), invalid ? 0 : 1);
      }
      if (invalid) {
        pfc_config_invalid = true;
        std::cerr << "INVALID PFC CONFIGURATION: switch " << i << " buffer "
                  << this_buffer_bytes << " bytes <= headroom " << sw->m_mmu->total_hdrm
                  << " + reserve " << sw->m_mmu->total_rsrv << " bytes; the pause "
                  << "threshold (buffer - headroom - reserve) would wrap to "
                  << (uint32_t)((uint32_t)this_buffer_bytes - sw->m_mmu->total_hdrm -
                                sw->m_mmu->total_rsrv)
                  << " >> shift" << std::endl;
        for (uint32_t j = 1; j < sw->GetNDevices(); j++) {
          Ptr<QbbNetDevice> dev = DynamicCast<QbbNetDevice>(sw->GetDevice(j));
          std::cerr << "  port " << j << " rate " << dev->GetDataRate().GetBitRate()
                    << " bit/s delay "
                    << DynamicCast<QbbChannel>(dev->GetChannel())->GetDelay().GetTimeStep()
                    << " ns headroom " << sw->m_mmu->headroom[j] << " bytes" << std::endl;
        }
      }
    }
  }
  if (link_report) {
    fflush(link_report);
  }
  if (pfc_config_invalid) {
    if (pfc_threshold_validation == "STRICT") {
      std::cerr << "PFC_THRESHOLD_VALIDATION STRICT: refusing to run." << std::endl;
      if (link_report)
        fclose(link_report);
      return false;
    }
    std::cerr << "PFC_THRESHOLD_VALIDATION LEGACY: continuing with the wrapped "
              << "threshold, as the original backend did." << std::endl;
  }

#if ENABLE_QP
  FILE *fct_output = fopen(fct_output_file.c_str(), "w");
  std::cout << "QP is enabled " << std::endl;
  //
  // install RDMA driver
  //
  for (uint32_t i = 0; i < node_num; i++) {
    if (n.Get(i)->GetNodeType() == 0) { // is server
      // create RdmaHw
      Ptr<RdmaHw> rdmaHw = CreateObject<RdmaHw>();
      rdmaHw->SetAttribute("ClampTargetRate", BooleanValue(clamp_target_rate));
      rdmaHw->SetAttribute("AlphaResumInterval",
                           DoubleValue(alpha_resume_interval));
      rdmaHw->SetAttribute("RPTimer", DoubleValue(rp_timer));
      rdmaHw->SetAttribute("FastRecoveryTimes",
                           UintegerValue(fast_recovery_times));
      rdmaHw->SetAttribute("EwmaGain", DoubleValue(ewma_gain));
      rdmaHw->SetAttribute("RateAI", DataRateValue(DataRate(rate_ai)));
      rdmaHw->SetAttribute("RateHAI", DataRateValue(DataRate(rate_hai)));
      rdmaHw->SetAttribute("L2BackToZero", BooleanValue(l2_back_to_zero));
      rdmaHw->SetAttribute("L2ChunkSize", UintegerValue(l2_chunk_size));
      rdmaHw->SetAttribute("L2AckInterval", UintegerValue(l2_ack_interval));
      rdmaHw->SetAttribute("CcMode", UintegerValue(cc_mode));
      rdmaHw->SetAttribute("RateDecreaseInterval",
                           DoubleValue(rate_decrease_interval));
      rdmaHw->SetAttribute("MinRate", DataRateValue(DataRate(min_rate)));
      rdmaHw->SetAttribute("Mtu", UintegerValue(packet_payload_size));
      rdmaHw->SetAttribute("MiThresh", UintegerValue(mi_thresh));
      rdmaHw->SetAttribute("VarWin", BooleanValue(var_win));
      rdmaHw->SetAttribute("FastReact", BooleanValue(fast_react));
      rdmaHw->SetAttribute("MultiRate", BooleanValue(multi_rate));
      rdmaHw->SetAttribute("SampleFeedback", BooleanValue(sample_feedback));
      rdmaHw->SetAttribute("TargetUtil", DoubleValue(u_target));
      rdmaHw->SetAttribute("RateBound", BooleanValue(rate_bound));
      rdmaHw->SetAttribute("DctcpRateAI",
                           DataRateValue(DataRate(dctcp_rate_ai)));
      rdmaHw->SetPintSmplThresh(pint_prob);
      rdmaHw->SetAttribute("TotalPauseTimes",
                           UintegerValue(nic_total_pause_time));
      // create and install RdmaDriver
      Ptr<RdmaDriver> rdma = CreateObject<RdmaDriver>();
      Ptr<Node> node = n.Get(i);
      rdma->SetNode(node);
      rdma->SetRdmaHw(rdmaHw);

      node->AggregateObject(rdma);
      rdma->Init();
      rdma->TraceConnectWithoutContext(
          "QpComplete", MakeBoundCallback(qp_finish, fct_output));
    }
  }
#endif

  // set ACK priority on hosts
  if (ack_high_prio)
    RdmaEgressQueue::ack_q_idx = 0;
  else
    RdmaEgressQueue::ack_q_idx = 3;

  // setup routing
  CalculateRoutes(n);
  SetRoutingEntries();

  //
  // get BDP and delay
  //
  maxRtt = maxBdp = 0;
  for (uint32_t i = 0; i < node_num; i++) {
    if (n.Get(i)->GetNodeType() != 0)
      continue;
    for (uint32_t j = 0; j < node_num; j++) {
      if (n.Get(j)->GetNodeType() != 0)
        continue;
      uint64_t delay = pairDelay[n.Get(i)][n.Get(j)];
      uint64_t txDelay = pairTxDelay[n.Get(i)][n.Get(j)];
      uint64_t rtt = delay * 2 + txDelay;
      uint64_t bw = pairBw[i][j];
      uint64_t bdp = rtt * bw / 1000000000 / 8;
      pairBdp[n.Get(i)][n.Get(j)] = bdp;
      pairRtt[i][j] = rtt;
      if (bdp > maxBdp)
        maxBdp = bdp;
      if (rtt > maxRtt)
        maxRtt = rtt;
      if (link_report && i != j)
        fprintf(link_report, "pair src=%u dst=%u one_way_delay_ns=%lu tx_delay_ns=%lu "
                "rtt_ns=%lu bottleneck_bps=%lu rtt_bdp_bytes=%lu\n",
                i, j, (unsigned long)delay, (unsigned long)txDelay, (unsigned long)rtt,
                (unsigned long)bw, (unsigned long)bdp);
    }
  }
  printf("maxRtt=%lu maxBdp=%lu\n", maxRtt, maxBdp);
  if (link_report) {
    fprintf(link_report, "global max_rtt_ns=%lu max_bdp_bytes=%lu has_win=%u global_t=%u\n",
            (unsigned long)maxRtt, (unsigned long)maxBdp, has_win, global_t);
    fclose(link_report);
    link_report = NULL;
  }

  //
  // setup switch CC
  //
  for (uint32_t i = 0; i < node_num; i++) {
    if (n.Get(i)->GetNodeType() == 1) { // switch
      Ptr<SwitchNode> sw = DynamicCast<SwitchNode>(n.Get(i));
      sw->SetAttribute("CcMode", UintegerValue(cc_mode));
      sw->SetAttribute("MaxRtt", UintegerValue(maxRtt));
    }
  }

  //
  // add trace
  //

  NodeContainer trace_nodes;
  for (uint32_t i = 0; i < trace_num; i++) {
    uint32_t nid;
    tracef >> nid;
    if (nid >= n.GetN()) {
      continue;
    }
    trace_nodes = NodeContainer(trace_nodes, n.Get(nid));
  }

  FILE *trace_output = fopen(trace_output_file.c_str(), "w");
  if (enable_trace)
    qbb.EnableTracing(trace_output, trace_nodes);

  // dump link speed to trace file
  {
    SimSetting sim_setting;
    for (auto i : nbr2if) {
      for (auto j : i.second) {
        uint16_t node = i.first->GetId();
        uint8_t intf = j.second.idx;
        uint64_t bps =
            DynamicCast<QbbNetDevice>(i.first->GetDevice(j.second.idx))
                ->GetDataRate()
                .GetBitRate();
        sim_setting.port_speed[node][intf] = bps;
      }
    }
    sim_setting.win = maxBdp;
    sim_setting.Serialize(trace_output);
  }

  Ipv4GlobalRoutingHelper::PopulateRoutingTables();

  NS_LOG_INFO("Create Applications.");

  Time interPacketInterval = Seconds(0.0000005 / 2);
  // maintain port number for each host
  for (uint32_t i = 0; i < node_num; i++) {
    if (n.Get(i)->GetNodeType() == 0)
      for (uint32_t j = 0; j < node_num; j++) {
        if (n.Get(j)->GetNodeType() == 0)
          portNumber[i][j] = 10000; // each host pair use port number from 10000
      }
  }
  flow_input.idx = -1;

  topof.close();
  tracef.close();

  // schedule link down
  if (link_down_time > 0) {
    Simulator::Schedule(Seconds(2) + MicroSeconds(link_down_time),
                        &TakeDownLink, n, n.Get(link_down_A),
                        n.Get(link_down_B));
  }

  // schedule buffer monitor
  FILE *qlen_output = fopen(qlen_mon_file.c_str(), "w");
  if (trace_level >= 1)
    Simulator::Schedule(NanoSeconds(qlen_mon_start), &monitor_buffer, qlen_output,
                        &n);

  // schedule state snapshots
  for (uint64_t t : state_snapshot_times)
    Simulator::Schedule(NanoSeconds(t), &write_state_snapshot, t);

  return true;
}
