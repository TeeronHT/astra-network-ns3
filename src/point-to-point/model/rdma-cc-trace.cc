#include <ns3/simulator.h>
#include "rdma-cc-trace.h"
#include "rdma-queue-pair.h"

namespace ns3 {

int RdmaCcTrace::mode = 0;
FILE* RdmaCcTrace::file = NULL;
uint32_t RdmaCcTrace::cc_mode = 0;

uint64_t RdmaCounters::ecn_marks = 0;
uint64_t RdmaCounters::cnp_acks = 0;
uint64_t RdmaCounters::mmu_drops = 0;
uint64_t RdmaCounters::pfc_pause_frames = 0;
uint64_t RdmaCounters::pfc_resume_frames = 0;
uint64_t RdmaCounters::qps_created = 0;
uint64_t RdmaCounters::qps_destroyed = 0;
uint64_t RdmaCounters::messages_posted = 0;
uint64_t RdmaCounters::messages_completed = 0;

void RdmaCcTrace::Open(const std::string& path, int trace_mode){
	mode = trace_mode;
	if (mode <= 0)
		return;
	file = fopen(path.c_str(), "w");
	if (!file){
		fprintf(stderr, "RdmaCcTrace: cannot open %s\n", path.c_str());
		mode = 0;
		return;
	}
	fprintf(file, "time_ns,event,conn_id,qp_id,src,dst,sport,dport,cc_mode,rate_bps,"
	              "target_rate_bps,alpha,hp_cur_rate_bps,window_bytes,snd_nxt,snd_una,"
	              "size_bytes,extra\n");
}

void RdmaCcTrace::Close(){
	if (file){
		fflush(file);
		fclose(file);
		file = NULL;
	}
	mode = 0;
}

void RdmaCcTrace::Log(const char* event, Ptr<RdmaQueuePair> qp, uint64_t extra){
	if (!file || !qp)
		return;
	uint64_t window = 0;
	if (cc_mode == 3)
		window = qp->HpGetCurWin();
	else
		window = qp->GetWin();
	fprintf(file, "%lu,%s,%lu,%lu,%u,%u,%u,%u,%u,%lu,%lu,%.9f,%lu,%lu,%lu,%lu,%lu,%lu\n",
	        (unsigned long)Simulator::Now().GetTimeStep(), event,
	        (unsigned long)qp->m_conn_id, (unsigned long)qp->m_qp_id,
	        qp->m_src, qp->m_dest, qp->sport, qp->dport, cc_mode,
	        (unsigned long)qp->m_rate.GetBitRate(),
	        (unsigned long)qp->mlx.m_targetRate.GetBitRate(),
	        qp->mlx.m_alpha,
	        (unsigned long)qp->hp.m_curRate.GetBitRate(),
	        (unsigned long)window,
	        (unsigned long)qp->snd_nxt, (unsigned long)qp->snd_una,
	        (unsigned long)qp->m_size, (unsigned long)extra);
}

void RdmaCcTrace::LogRx(const char* event, uint32_t node, uint32_t sip, uint32_t dip,
                        uint16_t sport, uint16_t dport, uint64_t extra){
	if (!file)
		return;
	// Receiver events have no sender QP; src/dst are derived from the
	// addresses the same way the frontend does (node id = bits 8..23).
	uint32_t src = (sip >> 8) & 0xffff, dst = (dip >> 8) & 0xffff;
	fprintf(file, "%lu,%s,0,0,%u,%u,%u,%u,%u,0,0,0,0,0,0,0,0,%lu\n",
	        (unsigned long)Simulator::Now().GetTimeStep(), event,
	        src, dst, sport, dport, cc_mode, (unsigned long)extra);
	(void)node;
}

} // namespace ns3
