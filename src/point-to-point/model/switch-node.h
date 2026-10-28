#ifndef SWITCH_NODE_H
#define SWITCH_NODE_H

#include <unordered_map>
#include <ns3/node.h>
#include "qbb-net-device.h"
#include "switch-mmu.h"
#include "pint.h"

namespace ns3 {

class Packet;

class SwitchNode : public Node{
	static const uint32_t pCnt = 1025;	// Number of ports used
	static const uint32_t qCnt = 8;	// Number of queues/priorities used
	uint32_t m_ecmpSeed;
	std::unordered_map<uint32_t, std::vector<int> > m_rtTable; // map from ip address (u32) to possible ECMP port (index of dev)

	// monitor of PFC
	uint32_t m_bytes[pCnt][pCnt][qCnt]; // m_bytes[inDev][outDev][qidx] is the bytes from inDev enqueued for outDev at qidx
	
	uint64_t m_txBytes[pCnt]; // counter of tx bytes

	uint32_t m_lastPktSize[pCnt];
	uint64_t m_lastPktTs[pCnt]; // ns
	double m_u[pCnt];

protected:
	bool m_ecnEnabled;
	bool m_pfcEnabled; // generate PAUSE/RESUME frames (default true; ENABLE_PFC 0 clears it)
	uint32_t m_ccMode;
	uint64_t m_maxRtt;

	uint32_t m_ackHighPrio; // set high priority for ACK/NACK

public:
	// observational counters (never read by the forwarding or marking logic)
	uint64_t m_ecnMarks[pCnt];   // packets marked CE per egress port
	uint64_t m_mmuDrops[pCnt];   // ingress-admission drops per ingress port
	uint64_t m_pfcPauseTx[pCnt]; // PAUSE frames generated per ingress port
	uint64_t m_pfcResumeTx[pCnt];

	// Return the egress interface this switch would choose for a UDP data
	// packet with the given 4-tuple, using exactly the ECMP hash of
	// GetOutDev. -1 when the destination has no entry. Does not touch state.
	int LookupDataNextHop(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport) const;
	uint32_t GetEcmpSeed() const { return m_ecmpSeed; }
	uint64_t GetTxBytes(uint32_t port) const { return port < pCnt ? m_txBytes[port] : 0; }

private:
	int GetOutDev(Ptr<const Packet>, CustomHeader &ch);
	void SendToDev(Ptr<Packet>p, CustomHeader &ch);
	static uint32_t EcmpHash(const uint8_t* key, size_t len, uint32_t seed);
	void CheckAndSendPfc(uint32_t inDev, uint32_t qIndex);
	void CheckAndSendResume(uint32_t inDev, uint32_t qIndex);
public:
	Ptr<SwitchMmu> m_mmu;

	static TypeId GetTypeId (void);
	SwitchNode();
	void SetEcmpSeed(uint32_t seed);
	void AddTableEntry(Ipv4Address &dstAddr, uint32_t intf_idx);
	void ClearTable();
	bool SwitchReceiveFromDevice(Ptr<NetDevice> device, Ptr<Packet> packet, CustomHeader &ch);
	void SwitchNotifyDequeue(uint32_t ifIndex, uint32_t qIndex, Ptr<Packet> p);

	// for approximate calc in PINT
	int logres_shift(int b, int l);
	int log2apprx(int x, int b, int m, int l); // given x of at most b bits, use most significant m bits of x, calc the result in l bits
};

} /* namespace ns3 */

#endif /* SWITCH_NODE_H */
