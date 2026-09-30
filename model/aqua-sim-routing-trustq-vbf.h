/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef AQUA_SIM_ROUTING_TRUSTQ_VBF_H
#define AQUA_SIM_ROUTING_TRUSTQ_VBF_H

#include "aqua-sim-routing-vbf.h"
#include "ns3/random-variable-stream.h"
#include <map>
#include <utility>
#include <vector>

namespace ns3 {

class AquaSimTrustQVBF : public AquaSimVBF
{
public:
  AquaSimTrustQVBF ();
  static TypeId GetTypeId (void);

  virtual bool Recv (Ptr<Packet> packet, const Address &dest, uint16_t protocolNumber);

  double   GetSelfTrust (void) const { return m_selfTrust; }
  uint32_t GetTimesEligible (void) const { return m_timesEligible; }
  uint32_t GetTimesForwarded (void) const { return m_timesForwarded; }

  bool IsMalicious (void) const { return m_isMalicious; }

  uint32_t GetTimesDropped (void) const { return m_timesDropped; }

  double GetObservedTrust (AquaSimAddress node) { return GetNeighborObservedTrust (node); }

protected:
  virtual void DoDispose ();

private:
  uint32_t m_timesEligible;
  uint32_t m_timesForwarded;
  uint32_t m_timesDropped;
  double   m_selfTrust;

  std::map<AquaSimAddress, double> m_neighborTrust;

  double m_trustWeight;
  double m_energyWeight;
  double m_distanceWeight;
  double m_delayWeight;
  double m_trustDecay;
  double m_initialEnergyRef;
  double m_delayNormConstant;
  double m_priorityScale;

  bool     m_isMalicious;
  double   m_dropProbability;
  Ptr<UniformRandomVariable> m_dropRand;
  double m_attackStart;
  uint32_t m_attackMode;
  double m_rampTime;
  double m_envBaseline;
  bool AttackerWantsDrop ();

  struct WatchEntry {
    uint16_t fwdAddr;
    Vector fwdPos, tgtPos, pipeO;
    double expFire;
    AquaSimAddress watchedNode;   // the EXPECTED forwarder we are watching
    bool resolved;
    bool expectedValid;           // false if no expected forwarder predicted
  };
  std::map<std::pair<AquaSimAddress, uint32_t>, WatchEntry> m_pktWatch;
  std::map<AquaSimAddress, double> m_observedTrust;

  double GetNeighborObservedTrust (AquaSimAddress node);
  void UpdateObservedTrust (AquaSimAddress node, bool forwardedOnward, double weight = 1.0);

  // [PACT] Peer-referenced Attenuated Continuous Trust.
  // Replaces EAQTE's binary freeze (EE < xi_th => discard observation
  // entirely) with a continuous weight in [0,1], referenced against the
  // node's PEERS rather than a fixed global threshold. Motivation: our
  // measurements show EAQTE's freeze incidence tracks traffic VOLUME, not
  // adversarial intent -- the busiest honest relay absorbed the largest
  // share of freezes in every configuration tested, including with no
  // attacker present at all.
  double ComputeEnvScore (AquaSimAddress node);   // EE_ij, shared by both modes
  double ComputePactWeight (AquaSimAddress node);
  bool   m_pactEnabled;
  void ScheduleWatchTimeout (AquaSimAddress origSrc, uint32_t pkNum);
  void CheckWatchTimeout (AquaSimAddress origSrc, uint32_t pkNum);

  // Passive neighbour position table: address -> (position, last-heard time).
  bool m_paperHoldTime;
  struct NeighborInfo {
    Vector pos;
    double lastHeard;
    Vector prevPos;    // [EAQTE Step 1] one sample back, for velocity estimation
    double prevHeard;
    bool   hasPrev;
    std::vector<double> pHistory;   // [EAQTE Step 2] recent p(d_ij,m) samples
  };
  std::map<AquaSimAddress, NeighborInfo> m_neighborPos;

  // [EAQTE Step 1] Stability score SS_ij, Eq. (25)-(26) of Khoshvaght et al.,
  // Computer Standards & Interfaces 96 (2026) 104087.
  Vector EstimateNeighborVelocity (AquaSimAddress node);
  double ComputeStabilityScore (AquaSimAddress node);

  // [EAQTE Step 2] Channel quality CCQ_ij, Eq. (6)-(10) and (18), same paper.
  // f=25kHz, k=1.5, Eb/N0=20dB are literature defaults NOT stated in the
  // paper's text -- see chat record for justification. m_bits must match
  // PACKET_SIZE in the scratch file.
  double TransmissionProbability (double distanceMeters) const;
  double ComputeChannelQuality (AquaSimAddress node);

  double m_neighborStaleTime;     // seconds before a neighbour entry is ignored
  double m_observedTrustWeight;   // weight of observed trust in relay priority

  // Standalone re-implementations of AquaSimVBF::Projection / CalculateDelay
  // evaluated for an ARBITRARY position instead of "this node".
  double ProjectionFor (Ptr<Packet> pkt, Vector at);
  double PredictDelayFor (Ptr<Packet> pkt, Vector at);
  double FireTimeFor (Vector o, Vector f, Vector t, Vector at, double prioFactor);
  bool   PredictExpectedForwarder (Ptr<Packet> pkt, AquaSimAddress prevHop,
                                   AquaSimAddress &outAddr);

  void UpdateSelfTrust (bool forwardedThisOne);
  void UpdateNeighborTrust (AquaSimAddress sender, uint8_t reportedTrustByte);
  double GetNeighborTrust (AquaSimAddress sender);
  uint8_t SelfTrustAsByte () const;

  void ConsiderNewTrustAware (Ptr<Packet> pkt);
  void SetDelayTimerTrustAware (Ptr<Packet> pkt, double delay);
  void TimeoutTrustAware (Ptr<Packet> pkt);
};

}

#endif /* AQUA_SIM_ROUTING_TRUSTQ_VBF_H */
