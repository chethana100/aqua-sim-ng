/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
 
#include "aqua-sim-routing-trustq-vbf.h"
#include <cstdlib>
#include "aqua-sim-header-routing.h"
#include "aqua-sim-header.h"
#include "aqua-sim-pt-tag.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/mobility-model.h"
#include "../../../scratch/mcm-mobility-model.h"  // [EAQTE Step 3v2] for TriggerReversal()
#include <cmath>
#include <vector>
#include <algorithm>
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include <string>
 
namespace ns3 {
 
NS_LOG_COMPONENT_DEFINE ("AquaSimTrustQVBF");
NS_OBJECT_ENSURE_REGISTERED (AquaSimTrustQVBF);

static double
RxRangeMeters ()
{
  static double v = -1.0;
  if (v < 0.0)
    {
      const char *s = std::getenv ("RX_RANGE_M");
      v = (s != 0) ? std::atof (s) : 0.0;
      if (v < 0.0)
        {
          v = 0.0;
        }
      NS_LOG_UNCOND ("[RANGE] rx_range_m=" << v << " (0 = off)");
    }
  return v;
}


static double
EnvThreshold ()
{
  static double v = -1.0;
  if (v < 0.0)
    {
      const char *s = std::getenv ("EAQTE_XI");
      v = (s != 0) ? std::atof (s) : 0.3;
      if (v < 0.0)
        {
          v = 0.0;
        }
      NS_LOG_UNCOND ("[EAQTE] xi_th=" << v);
    }
  return v;
}


class CustodyTag : public Tag
{
public:
  static TypeId GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::TrustQCustodyTag").SetParent<Tag> ().AddConstructor<CustodyTag> ();
    return tid;
  }
  virtual TypeId GetInstanceTypeId (void) const { return GetTypeId (); }
  virtual uint32_t GetSerializedSize (void) const { return 4; }
  virtual void Serialize (TagBuffer i) const { i.WriteU16 (relay); i.WriteU16 (prev); }
  virtual void Deserialize (TagBuffer i) { relay = i.ReadU16 (); prev = i.ReadU16 (); }
  virtual void Print (std::ostream &os) const { os << relay << "<-" << prev; }
  uint16_t relay = 0;
  uint16_t prev = 0;
};
 
TypeId
AquaSimTrustQVBF::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::AquaSimTrustQVBF")
    .SetParent<AquaSimVBF> ()
    .AddConstructor<AquaSimTrustQVBF> ()
    .AddAttribute ("TrustWeight", "Weight (omega_1) of trust in the priority score. Default 0.35.",
      DoubleValue (0.35),
      MakeDoubleAccessor (&AquaSimTrustQVBF::m_trustWeight),
      MakeDoubleChecker<double> ())
    .AddAttribute ("EnergyWeight", "Weight (omega_2) of residual energy in the priority score. Default 0.30.",
      DoubleValue (0.30),
      MakeDoubleAccessor (&AquaSimTrustQVBF::m_energyWeight),
      MakeDoubleChecker<double> ())
    .AddAttribute ("DistanceWeight", "Weight (omega_3) of distance-progress in the priority score. Default 0.20.",
      DoubleValue (0.20),
      MakeDoubleAccessor (&AquaSimTrustQVBF::m_distanceWeight),
      MakeDoubleChecker<double> ())
    .AddAttribute ("DelayWeight", "Weight (omega_4) penalizing HH-VBF's own base delay. Default 0.15.",
      DoubleValue (0.15),
      MakeDoubleAccessor (&AquaSimTrustQVBF::m_delayWeight),
      MakeDoubleChecker<double> ())
    .AddAttribute ("TrustDecay", "Decay factor (delta) for self- and neighbor-trust. Default 0.7.",
      DoubleValue (0.7),
      MakeDoubleAccessor (&AquaSimTrustQVBF::m_trustDecay),
      MakeDoubleChecker<double> ())
    .AddAttribute ("InitialEnergyRef", "Reference initial energy (J) for normalizing residual energy. Default 10000.",
      DoubleValue (10000.0),
      MakeDoubleAccessor (&AquaSimTrustQVBF::m_initialEnergyRef),
      MakeDoubleChecker<double> ())
    .AddAttribute ("DelayNormConstant", "Squashing constant (s) for normalizing HH-VBF base delay. Default 5.0.",
      DoubleValue (5.0),
      MakeDoubleAccessor (&AquaSimTrustQVBF::m_delayNormConstant),
      MakeDoubleChecker<double> ())
    .AddAttribute ("PriorityScale", "Caps how much the priority score can perturb HH-VBF's own delay. Default 0.2.",
      DoubleValue (0.2),
      MakeDoubleAccessor (&AquaSimTrustQVBF::m_priorityScale),
      MakeDoubleChecker<double> ())
    .AddAttribute ("PactEnabled",
      "[PACT] If true, replace EAQTE's binary freeze with peer-referenced "
      "continuous attenuation. Default false (reproduces published EAQTE).",
      BooleanValue (false),
      MakeBooleanAccessor (&AquaSimTrustQVBF::m_pactEnabled),
      MakeBooleanChecker ())
    .AddAttribute ("IsMalicious", "If true, this node drops packets it wins the right to forward.",
      BooleanValue (false),
      MakeBooleanAccessor (&AquaSimTrustQVBF::m_isMalicious),
      MakeBooleanChecker ())
    .AddAttribute ("DropProbability", "Probability of dropping instead of forwarding, if malicious.",
      DoubleValue (0.0),
      MakeDoubleAccessor (&AquaSimTrustQVBF::m_dropProbability),
      MakeDoubleChecker<double> (0.0, 1.0))
    .AddAttribute ("PaperHoldTime", "Use HH-VBF paper holding time sqrt(a)*T + (R-d)/v",
      BooleanValue (false),
      MakeBooleanAccessor (&AquaSimTrustQVBF::m_paperHoldTime),
      MakeBooleanChecker ())
    .AddAttribute ("ObservedTrustWeight",
      "Weight of observed (watchdog) trust in relay priority; 0 disables it",
      DoubleValue (0.0),
      MakeDoubleAccessor (&AquaSimTrustQVBF::m_observedTrustWeight),
      MakeDoubleChecker<double> (0.0, 1.0))
    .AddAttribute ("AttackStart",
      "Time (s) at which malicious nodes begin dropping; honest before it",
      DoubleValue (0.0),
      MakeDoubleAccessor (&AquaSimTrustQVBF::m_attackStart),
      MakeDoubleChecker<double> (0.0))
    .AddAttribute ("AttackMode",
      "0 = plain, 1 = environment-masked, 2 = ramped",
      UintegerValue (0),
      MakeUintegerAccessor (&AquaSimTrustQVBF::m_attackMode),
      MakeUintegerChecker<uint32_t> (0, 2))
    .AddAttribute ("RampTime",
      "Seconds over which a ramped attacker reaches DropProbability",
      DoubleValue (600.0),
      MakeDoubleAccessor (&AquaSimTrustQVBF::m_rampTime),
      MakeDoubleChecker<double> (0.0))
  ;
  return tid;
}
 
AquaSimTrustQVBF::AquaSimTrustQVBF ()
  : m_timesEligible (0),
    m_timesForwarded (0),
    m_timesDropped (0),
    m_selfTrust (0.5),
    m_trustWeight (0.35),
    m_energyWeight (0.30),
    m_distanceWeight (0.20),
    m_delayWeight (0.15),
    m_trustDecay (0.7),
    m_initialEnergyRef (10000.0),
    m_delayNormConstant (5.0),
    m_priorityScale (0.2),
    m_isMalicious (false),
    m_pactEnabled (false),
    m_dropProbability (0.0),
    m_neighborStaleTime (60.0),
    m_observedTrustWeight (0.0)
{
  NS_LOG_FUNCTION (this);
  m_dropRand = CreateObject<UniformRandomVariable> ();
  m_paperHoldTime = false;
  m_attackStart = 0.0;
  m_attackMode = 0;
  m_rampTime = 600.0;
  m_envBaseline = -1.0;
}
 
void
AquaSimTrustQVBF::DoDispose ()
{
  m_neighborTrust.clear ();
  m_observedTrust.clear ();
  m_pktWatch.clear ();
  AquaSimVBF::DoDispose ();
}
 
uint8_t
AquaSimTrustQVBF::SelfTrustAsByte () const
{
  double clamped = m_selfTrust;
  if (clamped < 0.0) clamped = 0.0;
  if (clamped > 1.0) clamped = 1.0;
  return static_cast<uint8_t> (clamped * 100.0);
}
 
void
AquaSimTrustQVBF::UpdateSelfTrust (bool forwardedThisOne)
{
  m_timesEligible++;
  if (forwardedThisOne)
    {
      m_timesForwarded++;
    }
  double observation = forwardedThisOne ? 1.0 : 0.0;
  m_selfTrust = m_trustDecay * m_selfTrust + (1.0 - m_trustDecay) * observation;
}
 
void
AquaSimTrustQVBF::UpdateNeighborTrust (AquaSimAddress sender, uint8_t reportedTrustByte)
{
  double reported = static_cast<double> (reportedTrustByte) / 100.0;
  double prior = 0.5;
  std::map<AquaSimAddress, double>::iterator it = m_neighborTrust.find (sender);
  if (it != m_neighborTrust.end ())
    {
      prior = it->second;
    }
  double updated = m_trustDecay * prior + (1.0 - m_trustDecay) * reported;
  m_neighborTrust[sender] = updated;
}
 
double
AquaSimTrustQVBF::GetNeighborTrust (AquaSimAddress sender)
{
  std::map<AquaSimAddress, double>::iterator it = m_neighborTrust.find (sender);
  if (it != m_neighborTrust.end ())
    {
      return it->second;
    }
  return 0.5;
}
 
// [OBSERVER TRUST -- Phase A]
double
AquaSimTrustQVBF::GetNeighborObservedTrust (AquaSimAddress node)
{
  std::map<AquaSimAddress, double>::iterator it = m_observedTrust.find (node);
  if (it != m_observedTrust.end ())
    {
      return it->second;
    }
  return 0.5;
}
 
void
AquaSimTrustQVBF::UpdateObservedTrust (AquaSimAddress node, bool forwardedOnward, double weight)
{
  NS_LOG_UNCOND ("[EAQTE-ATTEMPT] node=" << node);
  double prior = GetNeighborObservedTrust (node);
  double observation = forwardedOnward ? 1.0 : 0.0;

  // [PACT] weight in [0,1] attenuates how far this single observation can
  // move the score. weight=1 is the original EAQTE update; weight=0 leaves
  // the score untouched (equivalent to a freeze); intermediate values move
  // it proportionally less. This is the continuous replacement for the
  // binary freeze.
  if (weight < 0.0) weight = 0.0;
  if (weight > 1.0) weight = 1.0;
  double effectiveAlpha = (1.0 - m_trustDecay) * weight;
  m_observedTrust[node] = (1.0 - effectiveAlpha) * prior + effectiveAlpha * observation;
}

// [PACT] EE_ij, factored out so the binary-freeze and PACT paths compute
// the environment score identically -- only what they DO with it differs.
double
AquaSimTrustQVBF::ComputeEnvScore (AquaSimAddress node)
{
  double ccq = ComputeChannelQuality (node);
  double ss = ComputeStabilityScore (node);
  if (!m_pactEnabled)
    {
      double dist = -1.0;
      double age = -1.0;
      std::map<AquaSimAddress, NeighborInfo>::iterator eit = m_neighborPos.find (node);
      if (eit != m_neighborPos.end ())
        {
          Vector mp = GetNetDevice ()->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
          double dx = mp.x - eit->second.pos.x;
          double dy = mp.y - eit->second.pos.y;
          double dz = mp.z - eit->second.pos.z;
          dist = std::sqrt (dx * dx + dy * dy + dz * dz);
          age = Simulator::Now ().GetSeconds () - eit->second.lastHeard;
        }
      NS_LOG_UNCOND ("[SSCCQ] node=" << node << " ss=" << ss << " ccq=" << ccq
                     << " dist=" << dist << " age=" << age);
    }
  return 0.5 * ccq + 0.5 * ss;
}

// [PACT] Peer-referenced continuous weight.
// Key difference from EAQTE's fixed threshold: a node with low EE while
// its PEERS also have low EE is experiencing genuine shared turbulence and
// should NOT be singled out -- it keeps a high weight. Only a node whose
// EE is anomalously low RELATIVE to its current neighbourhood gets
// attenuated. This removes the volume bias: a busy relay is no longer
// penalised simply for generating more evaluation events in normal
// conditions.
double
AquaSimTrustQVBF::ComputePactWeight (AquaSimAddress node)
{
  double own = ComputeEnvScore (node);

  // Peer reference: median EE across all currently-known neighbours.
  std::vector<double> peerScores;
  for (std::map<AquaSimAddress, NeighborInfo>::iterator it = m_neighborPos.begin ();
       it != m_neighborPos.end (); ++it)
    {
      if (it->first == node)
        {
          continue;
        }
      peerScores.push_back (ComputeEnvScore (it->first));
    }

  if (peerScores.size () < 2)
    {
      // Too few peers to form a reference -- fall back to absolute EE,
      // mapped continuously rather than thresholded.
      return own;
    }

  std::sort (peerScores.begin (), peerScores.end ());
  double median;
  size_t n = peerScores.size ();
  if (n % 2 == 0)
    {
      median = 0.5 * (peerScores[n / 2 - 1] + peerScores[n / 2]);
    }
  else
    {
      median = peerScores[n / 2];
    }

  if (median <= 1e-9)
    {
      // Whole neighbourhood is unstable -- shared condition, don't single
      // this node out.
      return 1.0;
    }

  // Ratio > 1 means this node is doing BETTER than its peers -> full weight.
  double ratio = own / median;
  if (ratio > 1.0) ratio = 1.0;
  if (ratio < 0.0) ratio = 0.0;
  return ratio;
}
 
double
AquaSimTrustQVBF::ProjectionFor (Ptr<Packet> pkt, Vector at)
{
  VBHeader vbh; AquaSimHeader ash;
  pkt->RemoveHeader (ash); pkt->PeekHeader (vbh); pkt->AddHeader (ash);
  Vector t = vbh.GetExtraInfo ().t;
  Vector o = m_hopByHop ? vbh.GetExtraInfo ().f : vbh.GetExtraInfo ().o;
  double wx = t.x - o.x, wy = t.y - o.y, wz = t.z - o.z;
  double vx = at.x - o.x, vy = at.y - o.y, vz = at.z - o.z;
  double cx = vy * wz - vz * wy;
  double cy = vz * wx - vx * wz;
  double cz = vx * wy - vy * wx;
  double area = std::sqrt (cx * cx + cy * cy + cz * cz);
  double len = std::sqrt (wx * wx + wy * wy + wz * wz);
  if (len <= 0.0) return 0.0;
  return area / len;
}

double
AquaSimTrustQVBF::PredictDelayFor (Ptr<Packet> pkt, Vector at)
{
  VBHeader vbh; AquaSimHeader ash;
  pkt->RemoveHeader (ash); pkt->PeekHeader (vbh); pkt->AddHeader (ash);
  Vector f = vbh.GetExtraInfo ().f;
  Vector t = vbh.GetExtraInfo ().t;
  double dx = at.x - f.x, dy = at.y - f.y, dz = at.z - f.z;
  double dtx = t.x - f.x, dty = t.y - f.y, dtz = t.z - f.z;
  double dp = dx * dtx + dy * dty + dz * dtz;
  double d = std::sqrt (dx * dx + dy * dy + dz * dz);
  double l = std::sqrt (dtx * dtx + dty * dty + dtz * dtz);
  double ct = (d == 0.0 || l == 0.0) ? 0.0 : dp / (d * l);
  double range = m_device->GetPhy ()->GetTransRange ();
  double pr = ProjectionFor (pkt, at);
  return (pr / m_width) + ((range - d * ct) / range);
}

bool
AquaSimTrustQVBF::PredictExpectedForwarder (Ptr<Packet> pkt, AquaSimAddress prevHop, AquaSimAddress &outAddr)
{
  VBHeader vbh; AquaSimHeader ash;
  pkt->RemoveHeader (ash); pkt->PeekHeader (vbh); pkt->AddHeader (ash);
  Vector f = vbh.GetExtraInfo ().f;
  double range = m_device->GetPhy ()->GetTransRange ();
  double now = Simulator::Now ().GetSeconds ();
  bool found = false;
  double bestDelay = 0.0;
  std::map<AquaSimAddress, NeighborInfo>::iterator it;
  for (it = m_neighborPos.begin (); it != m_neighborPos.end (); ++it)
    {
      if (it->first == prevHop) continue;
      if (it->first == AquaSimAddress::ConvertFrom (GetNetDevice ()->GetAddress ())) continue;
      if (now - it->second.lastHeard > m_neighborStaleTime) continue;
      Vector np = it->second.pos;
      double sep = std::sqrt ((np.x - f.x) * (np.x - f.x) + (np.y - f.y) * (np.y - f.y) + (np.z - f.z) * (np.z - f.z));
      if (sep > range) continue;
      if (ProjectionFor (pkt, np) > m_width) continue;
      double base = PredictDelayFor (pkt, np);
      if (base < 0.0) base = 0.0;
      if (base > m_priority) continue;
      double hh = m_paperHoldTime ? std::sqrt (base) * DELAY + (range - sep) / ns3::SOUND_SPEED_IN_WATER : std::sqrt (base) * DELAY + 2.0 * (sep - range) / ns3::SOUND_SPEED_IN_WATER;
      if (hh < 0.0) hh = 0.0;
      double dly = sep / ns3::SOUND_SPEED_IN_WATER + hh * (1.0 - 0.5 * m_priorityScale);
      if (!found || dly < bestDelay) { bestDelay = dly; outAddr = it->first; found = true; }
    }
  return found;
}

double
AquaSimTrustQVBF::FireTimeFor (Vector o, Vector f, Vector t, Vector at, double prioFactor)
{
  double cs = ns3::SOUND_SPEED_IN_WATER;
  double range = m_device->GetPhy ()->GetTransRange ();
  double sx = at.x - f.x, sy = at.y - f.y, sz = at.z - f.z;
  double sep = std::sqrt (sx * sx + sy * sy + sz * sz);
  if (sep > range) return -1.0;
  double wx = t.x - o.x, wy = t.y - o.y, wz = t.z - o.z;
  double vx = at.x - o.x, vy = at.y - o.y, vz = at.z - o.z;
  double cx = vy * wz - vz * wy, cy = vz * wx - vx * wz, cz = vx * wy - vy * wx;
  double wl = std::sqrt (wx * wx + wy * wy + wz * wz);
  double proj = (wl > 0.0) ? std::sqrt (cx * cx + cy * cy + cz * cz) / wl : 0.0;
  if (proj > m_width) return -2.0;
  double tx = t.x - f.x, ty = t.y - f.y, tz = t.z - f.z;
  double tl = std::sqrt (tx * tx + ty * ty + tz * tz);
  double ct = (sep == 0.0 || tl == 0.0) ? 0.0 : (sx * tx + sy * ty + sz * tz) / (sep * tl);
  double base = proj / m_width + (range - sep * ct) / range;
  if (base < 0.0) base = 0.0;
  if (base > m_priority) return -3.0;
  double hh = m_paperHoldTime ? std::sqrt (base) * DELAY + (range - sep) / cs : std::sqrt (base) * DELAY + 2.0 * (sep - range) / cs;
  if (hh < 0.0) hh = 0.0;
  return sep / cs + hh * (1.0 - prioFactor * m_priorityScale);
}

bool
AquaSimTrustQVBF::AttackerWantsDrop ()
{
  double now = Simulator::Now ().GetSeconds ();
  double p = m_dropProbability;
  if (m_attackMode == 1)
    {
      // environment-masked: drop only while the local neighbourhood is thinner
      // than this node's own running average (environmental loss already high)
      uint32_t n = 0;
      for (std::map<AquaSimAddress, NeighborInfo>::iterator it = m_neighborPos.begin ();
           it != m_neighborPos.end (); ++it)
        {
          if (now - it->second.lastHeard <= m_neighborStaleTime)
            {
              n++;
            }
        }
      if (m_envBaseline < 0.0)
        {
          m_envBaseline = n;
        }
      bool degraded = n < m_envBaseline;
      m_envBaseline = 0.95 * m_envBaseline + 0.05 * n;
      if (!degraded)
        {
          return false;
        }
    }
  else if (m_attackMode == 2 && m_rampTime > 0.0)
    {
      // ramped: no sharp change point for a detector to find
      double r = (now - m_attackStart) / m_rampTime;
      if (r < 1.0)
        {
          p *= r;
        }
    }
  return m_dropRand->GetValue () < p;
}

// [EAQTE Step 1] Velocity of a neighbour, estimated from two overheard
// positions. Returns (0,0,0) if we don't yet have two samples for it --
// callers must treat that as "no estimate available", not "stationary".
Vector
AquaSimTrustQVBF::EstimateNeighborVelocity (AquaSimAddress node)
{
  std::map<AquaSimAddress, NeighborInfo>::iterator it = m_neighborPos.find (node);
  if (it == m_neighborPos.end () || !it->second.hasPrev)
    {
      return Vector (0, 0, 0);
    }
  double dt = it->second.lastHeard - it->second.prevHeard;
  if (dt < 0.05)  // [FIX] guard against near-simultaneous overhears
    {              // inflating velocity via division by a tiny dt
      return Vector (0, 0, 0);
    }
  return Vector ((it->second.pos.x - it->second.prevPos.x) / dt,
                  (it->second.pos.y - it->second.prevPos.y) / dt,
                  (it->second.pos.z - it->second.prevPos.z) / dt);
}

// [EAQTE Step 1] SS_ij = 0.5*(1 + VSIM(Vi,Vj)), Eq. (25)-(26),
// Khoshvaght et al., Computer Standards & Interfaces 96 (2026) 104087.
// Paper uses 2D (x,y) velocity components only.
double
AquaSimTrustQVBF::ComputeStabilityScore (AquaSimAddress node)
{
  Ptr<MobilityModel> selfModel = GetNetDevice ()->GetNode ()->GetObject<MobilityModel> ();
  Vector vi = selfModel->GetVelocity ();
  Vector vj = EstimateNeighborVelocity (node);

  double normI = std::sqrt (vi.x * vi.x + vi.y * vi.y);
  double normJ = std::sqrt (vj.x * vj.x + vj.y * vj.y);

  if (normI < 1e-6 || normJ < 1e-6)
    {
      // No reliable velocity estimate yet (e.g. first time hearing this
      // neighbour) -- neutral midpoint, matches SS_ij's own [0,1] range.
      return 0.5;
    }

  double vsim = (vi.x * vj.x + vi.y * vj.y) / (normI * normJ);
  if (vsim > 1.0) vsim = 1.0;
  if (vsim < -1.0) vsim = -1.0;

  return 0.5 * (1.0 + vsim);
}

// [EAQTE Step 2] p(d_ij,m), Eqs. (6)-(10), Khoshvaght et al., Computer
// Standards & Interfaces 96 (2026) 104087. Constants not stated in the
// paper's text -- literature defaults, see chat record for justification:
//   f = 25 kHz    (WHOI micro-modem centre frequency, paper's own ref [48])
//   k = 1.5       ("practical spreading", Stojanovic, paper's own ref [47])
//   Eb/N0 = 20 dB (100 linear) -- weakest-justified of the three constants.
double
AquaSimTrustQVBF::TransmissionProbability (double distanceMeters) const
{
  const double f_kHz = 25.0;
  const double k = 1.5;
  const double EbN0_linear = 10000.0;  // 40 dB -- recalibrated: paper's own
                                         // simulation used a 100m comm range
                                         // (Table 2); 20dB produced near-zero
                                         // p(d,m) at this project's 1000m
                                         // range. 40dB gives p(1000m)~0.94,
                                         // matching a link genuinely designed
                                         // to work reliably at that range.
  const double m_bits = 640.0;        // 80 bytes -- MUST match PACKET_SIZE
                                        // in uwsn-trustq-attack.cc

  double d_km = distanceMeters / 1000.0;
  if (d_km <= 0.0)
    {
      return 1.0;  // co-located -- treat as a perfect channel
    }

  // Thorp absorption coefficient, Eq. (7). f in kHz, result in dB/km.
  double f2 = f_kHz * f_kHz;
  double alpha_dB = 0.11 * (f2 / (1.0 + f2))
                   + 44.0 * (f2 / (4100.0 + f2))
                   + 2.75e-4 * f2
                   + 0.003;
  double alpha_linear = std::pow (10.0, alpha_dB / 10.0);

  double A = std::pow (d_km, k) * std::pow (alpha_linear, d_km);       // Eq. (6)
  double snr = EbN0_linear / A;                                        // Eq. (8)
  double pe = 0.5 * (1.0 - std::sqrt (snr / (1.0 + snr)));              // Eq. (9)
  double p = std::pow (1.0 - pe, m_bits);                               // Eq. (10)

  if (p < 0.0) p = 0.0;
  if (p > 1.0) p = 1.0;
  return p;
}

// [EAQTE Step 2] CCQ_ij = p(d_ij,m) * (1 - tanh(var(p over recent samples))),
// Eq. (18). Variance formula matches Eq. (14)-(17) exactly (population
// variance, not sample variance).
double
AquaSimTrustQVBF::ComputeChannelQuality (AquaSimAddress node)
{
  std::map<AquaSimAddress, NeighborInfo>::iterator it = m_neighborPos.find (node);
  if (it == m_neighborPos.end ())
    {
      return 0.5;  // no data yet -- neutral midpoint
    }

  Ptr<MobilityModel> selfModel = GetNetDevice ()->GetNode ()->GetObject<MobilityModel> ();
  Vector myPos = selfModel->GetPosition ();
  Vector theirPos = it->second.pos;
  double dx = myPos.x - theirPos.x;
  double dy = myPos.y - theirPos.y;
  double dz = myPos.z - theirPos.z;
  double distance = std::sqrt (dx * dx + dy * dy + dz * dz);

  double pNow = TransmissionProbability (distance);

  const size_t MAX_HISTORY = 10;
  it->second.pHistory.push_back (pNow);
  if (it->second.pHistory.size () > MAX_HISTORY)
    {
      it->second.pHistory.erase (it->second.pHistory.begin ());
    }

  if (it->second.pHistory.size () < 2)
    {
      return pNow;   // not enough samples for variance yet; tanh(0)=0 -> CCQ=p
    }

  double mean = 0.0;
  for (size_t i = 0; i < it->second.pHistory.size (); i++)
    {
      mean += it->second.pHistory[i];
    }
  mean /= it->second.pHistory.size ();

  double variance = 0.0;
  for (size_t i = 0; i < it->second.pHistory.size (); i++)
    {
      double diff = it->second.pHistory[i] - mean;
      variance += diff * diff;
    }
  variance /= it->second.pHistory.size ();

  double ccq = pNow * (1.0 - std::tanh (variance));
  if (ccq < 0.0) ccq = 0.0;
  if (ccq > 1.0) ccq = 1.0;
  return ccq;
}

void
AquaSimTrustQVBF::ScheduleWatchTimeout (AquaSimAddress origSrc, uint32_t pkNum)
{
  // 8s window: generous relative to HH-VBF's own contention delays, so we
  // don't falsely accuse a node still legitimately waiting its turn.
  Simulator::Schedule (Seconds (8.0), &AquaSimTrustQVBF::CheckWatchTimeout,
                        this, origSrc, pkNum);
}
 
void
AquaSimTrustQVBF::CheckWatchTimeout (AquaSimAddress origSrc, uint32_t pkNum)
{
  std::pair<AquaSimAddress, uint32_t> key (origSrc, pkNum);
  std::map<std::pair<AquaSimAddress, uint32_t>, WatchEntry>::iterator it = m_pktWatch.find (key);
  if (it == m_pktWatch.end ())
    {
      return; // already resolved and erased
    }
  if (!it->second.resolved)
    {
      // Never overheard again within the window -- suspected drop.
      if (it->second.expectedValid)
        {
          NS_LOG_UNCOND ("[OBSERVER] TIMEOUT exp=" << it->second.watchedNode);
        }
    }
  m_pktWatch.erase (it);
}
 
bool
AquaSimTrustQVBF::Recv (Ptr<Packet> packet, const Address &dest, uint16_t protocolNumber)
{
  NS_LOG_FUNCTION (this);
  AquaSimHeader ash;
  VBHeader vbh;
  AquaSimPtTag ptag;
  packet->RemoveHeader (ash);
 
  if (ash.GetNumForwards () <= 0)
    {
      ash.SetDirection (AquaSimHeader::DOWN);
      ash.SetNumForwards (1);
      ash.SetSAddr (AquaSimAddress::ConvertFrom (GetNetDevice ()->GetAddress ()));
      ash.SetDAddr (AquaSimAddress::ConvertFrom (dest));
      ash.SetTimeStamp (Simulator::Now ());
      ash.SetUId (packet->GetUid ());
 
      vbh.SetMessType (AS_DATA);
      vbh.SetSenderAddr (AquaSimAddress::ConvertFrom (GetNetDevice ()->GetAddress ()));
      vbh.SetForwardAddr (AquaSimAddress::ConvertFrom (GetNetDevice ()->GetAddress ()));
      vbh.SetTargetAddr (AquaSimAddress::ConvertFrom (dest));
      vbh.SetPkNum (packet->GetUid ());
      NS_LOG_UNCOND ("[EAQTE] TX src=" << AquaSimAddress::ConvertFrom (GetNetDevice ()->GetAddress ()).GetAsInt () << " pk=" << packet->GetUid () << " d=" << CalculateDistance (GetNetDevice ()->GetNode ()->GetObject<MobilityModel> ()->GetPosition (), m_targetPos) << " t=" << Simulator::Now ().GetSeconds ());
      vbh.SetDataType (SelfTrustAsByte ());
 
      Ptr<Object> sObject = GetNetDevice ()->GetNode ();
      Ptr<MobilityModel> sModel = sObject->GetObject<MobilityModel> ();
      vbh.SetOriginalSource (sModel->GetPosition ());
      vbh.SetExtraInfo_f (sModel->GetPosition ());
      vbh.SetExtraInfo_t (m_targetPos);
      vbh.SetExtraInfo_o (sModel->GetPosition ());
 
      packet->AddHeader (vbh);
    }
  else
    {
      packet->PeekHeader (vbh);
    }
 
  packet->AddHeader (ash);
 
  if (RxRangeMeters () > 0.0)
    {
      Vector rp = GetNetDevice ()->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
      Vector tp = vbh.GetExtraInfo ().f;
      double rdx = rp.x - tp.x;
      double rdy = rp.y - tp.y;
      double rdz = rp.z - tp.z;
      if (std::sqrt (rdx * rdx + rdy * rdy + rdz * rdz) > RxRangeMeters ())
        {
          static uint32_t rangeDrops = 0;
          rangeDrops++;
          if (rangeDrops % 20000 == 1)
            {
              NS_LOG_UNCOND ("[RANGE] dropped " << rangeDrops << " receptions beyond "
                             << RxRangeMeters () << " m");
            }
          packet = 0;
          return false;
        }
    }
  if (!m_enableRouting)
    {
      if (vbh.GetMessType () != AS_DATA)
        {
          packet = 0;
          return false;
        }
      if (vbh.GetSenderAddr () == GetNetDevice ()->GetAddress ())
        {
          ptag.SetPacketType (AquaSimPtTag::PT_UWVB);
          packet->ReplacePacketTag (ptag);
          MACprepare (packet);
          MACsend (packet, (m_rand->GetValue () * JITTER));
        }
      else if (vbh.GetTargetAddr () == GetNetDevice ()->GetAddress ())
        {
          DataForSink (packet);
        }
      return true;
    }
 
  vbf_neighborhood *hashPtr = PktTable.GetHash (vbh.GetSenderAddr (), vbh.GetPkNum ());
 
  if (hashPtr != NULL)
    {
      packet->PeekHeader (ash);
      UpdateNeighborTrust (ash.GetSAddr (), vbh.GetDataType ());
 
      // [OBSERVER TRUST] We've heard this exact packet before. If the
      // current transmitter differs from whoever we were watching, the
      // packet genuinely continued onward -- credit that node with a
      // real, observed forward (not a self-report).
      {
        NeighborInfo dni;
        dni.pos = vbh.GetExtraInfo ().f;
        dni.lastHeard = Simulator::Now ().GetSeconds ();
        std::map<AquaSimAddress, NeighborInfo>::iterator oldIt1 = m_neighborPos.find (vbh.GetForwardAddr ());
        if (oldIt1 != m_neighborPos.end ())
          {
            dni.prevPos = oldIt1->second.pos;
            dni.prevHeard = oldIt1->second.lastHeard;
            dni.hasPrev = true;
          }
        else
          {
            dni.hasPrev = false;
          }
        m_neighborPos[vbh.GetForwardAddr ()] = dni;
      }
      std::pair<AquaSimAddress, uint32_t> key (vbh.GetSenderAddr (), vbh.GetPkNum ());
      std::map<std::pair<AquaSimAddress, uint32_t>, WatchEntry>::iterator wit = m_pktWatch.find (key);
      bool hasTag = false, sameHop = false;
      if (wit != m_pktWatch.end ())
        {
          uint16_t me = vbh.GetForwardAddr ().GetAsInt ();
          ByteTagIterator bti = packet->GetByteTagIterator ();
          while (bti.HasNext ())
            {
              ByteTagIterator::Item item = bti.Next ();
              if (item.GetTypeId () != CustodyTag::GetTypeId ()) continue;
              CustodyTag ct;
              item.GetTag (ct);
              if (ct.relay != me) continue;
              hasTag = true;
              if (ct.prev == wit->second.fwdAddr) sameHop = true;
            }
          NS_LOG_UNCOND ("[OBSERVER] CUSTODY tag=" << hasTag << " same=" << sameHop);
        }
      if (wit != m_pktWatch.end () && !wit->second.resolved
          && wit->second.expectedValid && sameHop)
        {
          // FIX: credit the watched node ONLY when it is itself the one
          // we personally overhear retransmitting -- not merely "some
          // other node forwarded this packet." The prior (!=) condition
          // credited the watched node for ANY third party's honest relay,
          // letting an actual dropper be falsely exonerated whenever a
          // different eligible neighbor happened to forward the same
          // packet within the watch window.
          if (vbh.GetForwardAddr () == wit->second.watchedNode)
            {
              // [EAQTE Step 3] Freeze: skip the trust update entirely if
              // EE_ij < xi_th (0.3, paper's own stated value) for this
              // specific neighbor -- link/motion instability toward THIS
              // node means we cannot safely attribute this observation.
              if (m_pactEnabled)
                {
                  double w = ComputePactWeight (wit->second.watchedNode);
                  NS_LOG_UNCOND ("[PACT] node=" << wit->second.watchedNode << " weight=" << w);
                  UpdateObservedTrust (wit->second.watchedNode, true, w);
                }
              else
                {
                  double ee = ComputeEnvScore (wit->second.watchedNode);
                  NS_LOG_UNCOND ("[EE] node=" << wit->second.watchedNode << " ee=" << ee << " kind=direct");
                  if (ee < EnvThreshold ())
                    {
                      NS_LOG_UNCOND ("[EAQTE-FREEZE] node=" << wit->second.watchedNode << " EE=" << ee << " -- update skipped");
                    }
                  else
                    {
                      UpdateObservedTrust (wit->second.watchedNode, true);
                    }
                }
            }
          else
            {
              double actFire = FireTimeFor (wit->second.pipeO, wit->second.fwdPos, wit->second.tgtPos, vbh.GetExtraInfo ().f, 1.0);
              if (actFire < 0.0)
                {
                  NS_LOG_UNCOND ("[OBSERVER] MODELMISS exp=" << wit->second.watchedNode << " act=" << vbh.GetForwardAddr () << " why=" << actFire);
                }
              else if (actFire > wit->second.expFire + 0.005)
                {
                  if (m_pactEnabled)
                    {
                      double wBlamed = ComputePactWeight (wit->second.watchedNode);
                      double wCredited = ComputePactWeight (vbh.GetForwardAddr ());
                      NS_LOG_UNCOND ("[PACT] node=" << wit->second.watchedNode << " weight=" << wBlamed << " (blame)");
                      NS_LOG_UNCOND ("[PACT] node=" << vbh.GetForwardAddr () << " weight=" << wCredited << " (credit)");
                      UpdateObservedTrust (wit->second.watchedNode, false, wBlamed);
                      UpdateObservedTrust (vbh.GetForwardAddr (), true, wCredited);
                    }
                  else
                    {
                      double eeBlamed = ComputeEnvScore (wit->second.watchedNode);
                  NS_LOG_UNCOND ("[EE] node=" << wit->second.watchedNode << " ee=" << eeBlamed << " kind=blame");
                      double eeCredited = ComputeEnvScore (vbh.GetForwardAddr ());
                  NS_LOG_UNCOND ("[EE] node=" << vbh.GetForwardAddr () << " ee=" << eeCredited << " kind=backup");
                      if (eeBlamed < EnvThreshold ())
                        {
                          NS_LOG_UNCOND ("[EAQTE-FREEZE] node=" << wit->second.watchedNode << " EE=" << eeBlamed << " -- blame skipped");
                        }
                      else
                        {
                          UpdateObservedTrust (wit->second.watchedNode, false);
                        }
                      if (eeCredited < EnvThreshold ())
                        {
                          NS_LOG_UNCOND ("[EAQTE-FREEZE] node=" << vbh.GetForwardAddr () << " EE=" << eeCredited << " -- credit skipped");
                        }
                      else
                        {
                          UpdateObservedTrust (vbh.GetForwardAddr (), true);
                        }
                    }
                  NS_LOG_UNCOND ("[OBSERVER] DEVIATION exp=" << wit->second.watchedNode << " act=" << vbh.GetForwardAddr () << " gap=" << (actFire - wit->second.expFire));
                }
              else
                {
                  NS_LOG_UNCOND ("[OBSERVER] TIE exp=" << wit->second.watchedNode << " act=" << vbh.GetForwardAddr ());
                }
            }
          wit->second.resolved = true;
        }
 
      PktTable.PutInHash (vbh.GetSenderAddr (), vbh.GetPkNum (), vbh.GetExtraInfo ().f);
      packet = 0;
      return false;
    }
  else
    {
      packet->PeekHeader (ash);
      UpdateNeighborTrust (ash.GetSAddr (), vbh.GetDataType ());
 
      PktTable.PutInHash (vbh.GetSenderAddr (), vbh.GetPkNum (), vbh.GetExtraInfo ().f);
 
      // [OBSERVER TRUST] First time hearing this packet -- start watching
      // whether it continues onward from the node we just overheard.
      std::pair<AquaSimAddress, uint32_t> newKey (vbh.GetSenderAddr (), vbh.GetPkNum ());
      NeighborInfo ni;
      ni.pos = vbh.GetExtraInfo ().f;
      ni.lastHeard = Simulator::Now ().GetSeconds ();
      {
        std::map<AquaSimAddress, NeighborInfo>::iterator oldIt2 = m_neighborPos.find (vbh.GetForwardAddr ());
        if (oldIt2 != m_neighborPos.end ())
          {
            ni.prevPos = oldIt2->second.pos;
            ni.prevHeard = oldIt2->second.lastHeard;
            ni.hasPrev = true;
          }
        else
          {
            ni.hasPrev = false;
          }
      }
      m_neighborPos[vbh.GetForwardAddr ()] = ni;
      WatchEntry entry;
      AquaSimAddress expected;
      entry.expectedValid = PredictExpectedForwarder (packet, vbh.GetForwardAddr (), expected);
      entry.watchedNode = entry.expectedValid ? expected : vbh.GetForwardAddr ();
      entry.resolved = false;
      entry.fwdPos = vbh.GetExtraInfo ().f;
      entry.fwdAddr = vbh.GetForwardAddr ().GetAsInt ();
      entry.tgtPos = vbh.GetExtraInfo ().t;
      entry.pipeO = m_hopByHop ? vbh.GetExtraInfo ().f : vbh.GetExtraInfo ().o;
      entry.expFire = entry.expectedValid ? FireTimeFor (entry.pipeO, entry.fwdPos, entry.tgtPos, m_neighborPos[expected].pos, 0.0) : 0.0;
      m_pktWatch[newKey] = entry;
      NS_LOG_UNCOND ("[OBSERVER] watch prev=" << vbh.GetForwardAddr () << " sa=" << ash.GetSAddr () << " ev=" << entry.expectedValid << " exp=" << entry.watchedNode << " cands=" << m_neighborPos.size ());
      NS_LOG_UNCOND ("[EAQTE-SS] neighbor=" << vbh.GetForwardAddr () << " SS=" << ComputeStabilityScore (vbh.GetForwardAddr ()));
      NS_LOG_UNCOND ("[EAQTE-CCQ] neighbor=" << vbh.GetForwardAddr () << " CCQ=" << ComputeChannelQuality (vbh.GetForwardAddr ()));
      ScheduleWatchTimeout (vbh.GetSenderAddr (), vbh.GetPkNum ());
 
      Ptr<Object> sObject = GetNetDevice ()->GetNode ();
      Ptr<MobilityModel> sModel = sObject->GetObject<MobilityModel> ();
      Vector forwarder = vbh.GetExtraInfo ().f;
 
      packet->RemoveHeader (ash);
      packet->RemoveHeader (vbh);
      Vector d = Vector (sModel->GetPosition ().x - forwarder.x,
                          sModel->GetPosition ().y - forwarder.y,
                          sModel->GetPosition ().z - forwarder.z);
      vbh.SetExtraInfo_d (d);
      packet->AddHeader (vbh);
      packet->AddHeader (ash);
 
      if (vbh.GetMessType () == AS_DATA)
        {
          ConsiderNewTrustAware (packet);
        }
      else
        {
          ConsiderNew (packet);
        }
    }
 
  return true;
}
 
void
AquaSimTrustQVBF::ConsiderNewTrustAware (Ptr<Packet> pkt)
{
  NS_LOG_FUNCTION (this);
  AquaSimHeader ash;
  VBHeader vbh;
  pkt->RemoveHeader (ash);
  pkt->PeekHeader (vbh);
  pkt->AddHeader (ash);
 
  AquaSimAddress from_nodeAddr = vbh.GetSenderAddr ();
 
  if (GetNetDevice ()->GetAddress () == from_nodeAddr)
    {
      MACprepare (pkt);
      MACsend (pkt, 0);
      return;
    }
 
  if (GetNetDevice ()->GetAddress () == vbh.GetTargetAddr ())
    {
      NS_LOG_UNCOND ("[EAQTE] RX src=" << vbh.GetSenderAddr ().GetAsInt () << " pk=" << vbh.GetPkNum () << " t=" << Simulator::Now ().GetSeconds ());
      DataForSink (pkt);
      return;
    }
 
  if (IsCloseEnough (pkt))
    {
      Vector *p1 = new Vector[1];
      p1[0].x = vbh.GetExtraInfo ().f.x;
      p1[0].y = vbh.GetExtraInfo ().f.y;
      p1[0].z = vbh.GetExtraInfo ().f.z;
      double baseDelay = CalculateDelay (pkt, p1);
      delete[] p1;
 
      double d2 = (Distance (pkt) - m_device->GetPhy ()->GetTransRange ()) / ns3::SOUND_SPEED_IN_WATER;
      double hhvbfDelay = m_paperHoldTime
        ? (sqrt (baseDelay) * DELAY + (m_device->GetPhy ()->GetTransRange () - Distance (pkt)) / ns3::SOUND_SPEED_IN_WATER)
        : (sqrt (baseDelay) * DELAY + d2 * 2);
 
      double pht = GetNeighborObservedTrust (vbh.GetSenderAddr ());
      double trustTerm = (1.0 - m_observedTrustWeight) * m_selfTrust + m_observedTrustWeight * (1.0 - pht);
 
      double energyTerm = 0.5;
      if (m_device && m_device->EnergyModel ())
        {
          double residual = m_device->EnergyModel ()->GetEnergy ();
          energyTerm = residual / m_initialEnergyRef;
          if (energyTerm > 1.0) energyTerm = 1.0;
          if (energyTerm < 0.0) energyTerm = 0.0;
        }
 
      Ptr<Object> sObject = GetNetDevice ()->GetNode ();
      Ptr<MobilityModel> sModel = sObject->GetObject<MobilityModel> ();
      Vector myPos = sModel->GetPosition ();
      Vector origSrc = vbh.GetOriginalSource ();
      Vector target = m_targetPos;
 
      double distMeToSink = std::sqrt (
        (myPos.x - target.x) * (myPos.x - target.x) +
        (myPos.y - target.y) * (myPos.y - target.y) +
        (myPos.z - target.z) * (myPos.z - target.z));
      double distSrcToSink = std::sqrt (
        (origSrc.x - target.x) * (origSrc.x - target.x) +
        (origSrc.y - target.y) * (origSrc.y - target.y) +
        (origSrc.z - target.z) * (origSrc.z - target.z));
 
      double distanceTerm = 0.5;
      if (distSrcToSink > 0.0)
        {
          distanceTerm = 1.0 - (distMeToSink / distSrcToSink);
          if (distanceTerm > 1.0) distanceTerm = 1.0;
          if (distanceTerm < 0.0) distanceTerm = 0.0;
        }
 
      double delayTerm = hhvbfDelay / (hhvbfDelay + m_delayNormConstant);
 
      double priority = m_trustWeight * trustTerm
                       + m_energyWeight * energyTerm
                       + m_distanceWeight * distanceTerm
                       - m_delayWeight * delayTerm;
 
      if (priority < 0.0) priority = 0.0;
      if (priority > 1.0) priority = 1.0;
 
      double scaledPriority = priority * m_priorityScale;
 
      double finalDelay = hhvbfDelay * (1.0 - scaledPriority);
      if (finalDelay < 0.0)
        {
          finalDelay = 0.0;
        }
 
      SetDelayTimerTrustAware (pkt, finalDelay);
    }
  else
    {
      pkt = 0;
    }
}
 
void
AquaSimTrustQVBF::SetDelayTimerTrustAware (Ptr<Packet> pkt, double c)
{
  NS_LOG_FUNCTION (this << c);
  if (c < 0) c = 0;
  Simulator::Schedule (Seconds (c), &AquaSimTrustQVBF::TimeoutTrustAware, this, pkt);
}
 
void
AquaSimTrustQVBF::TimeoutTrustAware (Ptr<Packet> pkt)
{
  VBHeader vbh;
  AquaSimHeader ash;
  pkt->RemoveHeader (ash);
  pkt->PeekHeader (vbh);
  pkt->AddHeader (ash);
 
  if (vbh.GetMessType () != AS_DATA)
    {
      pkt = 0;
      return;
    }
 
  vbf_neighborhood *hashPtr = PktTable.GetHash (vbh.GetSenderAddr (), vbh.GetPkNum ());
 
  if (hashPtr == NULL)
    {
      return;
    }
 
  int num_neighbor = hashPtr->number;
 
  if (num_neighbor != 1)
    {
      if (num_neighbor == MAX_NEIGHBOR)
        {
          pkt = 0;
          return;
        }
 
      int i = 0;
      Vector *tp = new Vector[1];
      tp[0].x = hashPtr->neighbor[i].x;
      tp[0].y = hashPtr->neighbor[i].y;
      tp[0].z = hashPtr->neighbor[i].z;
      double tdelay = CalculateDelay (pkt, tp);
      i++;
      double c = 1;
      while (i < num_neighbor)
        {
          c = c * 2;
          tp[0].x = hashPtr->neighbor[i].x;
          tp[0].y = hashPtr->neighbor[i].y;
          tp[0].z = hashPtr->neighbor[i].z;
          double t2delay = CalculateDelay (pkt, tp);
          if (t2delay < tdelay)
            {
              tdelay = t2delay;
            }
          i++;
        }
      delete[] tp;
 
      if (tdelay <= (m_priority / c))
        {
          // [ATTACKER FIX -- drop counter] Only place a malicious drop is
          // observable via ground truth. times_eligible/times_forwarded
          // never see it, since UpdateSelfTrust is never called on this
          // path (by design). The observer-trust mechanism above is what
          // is SUPPOSED to catch this from the outside -- that's the test.
          if (m_isMalicious && Simulator::Now ().GetSeconds () >= m_attackStart && AttackerWantsDrop ())
            {
              // [EAQTE Step 3v2] Trigger a brief (1.5s) motion reversal on
              // THIS node's own mobility model, right at the moment of the
              // actual drop -- suppresses this node's SS_ij toward its
              // neighbours for a short window, targeting EE_ij < 0.3 to
              // freeze the trust judgment that would otherwise catch this
              // exact drop. No-op if AdversarialMotion was never enabled
              // or if mobility model isn't McmMobilityModel.
              Ptr<McmMobilityModel> selfMob = DynamicCast<McmMobilityModel> (GetNetDevice ()->GetNode ()->GetObject<MobilityModel> ());
              if (selfMob)
                {
                  selfMob->TriggerReversal (1.5);
                }
              m_timesDropped++; NS_LOG_UNCOND ("[OBSERVER] DROP node=" << AquaSimAddress::ConvertFrom (GetNetDevice ()->GetAddress ()) << " src=" << vbh.GetSenderAddr ().GetAsInt () << " pk=" << vbh.GetPkNum () << " t=" << Simulator::Now ().GetSeconds ());
              pkt = 0;
              return;
            }
          {
            VBHeader cvb; AquaSimHeader cash;
            pkt->RemoveHeader (cash); pkt->PeekHeader (cvb); pkt->AddHeader (cash);
            CustodyTag ct;
            ct.relay = AquaSimAddress::ConvertFrom (GetNetDevice ()->GetAddress ()).GetAsInt ();
            ct.prev = cvb.GetForwardAddr ().GetAsInt ();
            pkt->AddByteTag (ct);
          }
          MACprepare (pkt);
          MACsend (pkt, 0);
          UpdateSelfTrust (true);
        }
      else
        {
          pkt = 0;
        }
    }
  else
    {
      Vector *tp = new Vector[1];
      tp[0].x = vbh.GetExtraInfo ().f.x;
      tp[0].y = vbh.GetExtraInfo ().f.y;
      tp[0].z = vbh.GetExtraInfo ().f.z;
      double delay = CalculateDelay (pkt, tp);
      delete[] tp;
 
      if (delay <= m_priority)
        {
          if (m_isMalicious && Simulator::Now ().GetSeconds () >= m_attackStart && AttackerWantsDrop ())
            {
              // [EAQTE Step 3v2] Trigger a brief (1.5s) motion reversal on
              // THIS node's own mobility model, right at the moment of the
              // actual drop -- suppresses this node's SS_ij toward its
              // neighbours for a short window, targeting EE_ij < 0.3 to
              // freeze the trust judgment that would otherwise catch this
              // exact drop. No-op if AdversarialMotion was never enabled
              // or if mobility model isn't McmMobilityModel.
              Ptr<McmMobilityModel> selfMob = DynamicCast<McmMobilityModel> (GetNetDevice ()->GetNode ()->GetObject<MobilityModel> ());
              if (selfMob)
                {
                  selfMob->TriggerReversal (1.5);
                }
              m_timesDropped++; NS_LOG_UNCOND ("[OBSERVER] DROP node=" << AquaSimAddress::ConvertFrom (GetNetDevice ()->GetAddress ()) << " src=" << vbh.GetSenderAddr ().GetAsInt () << " pk=" << vbh.GetPkNum () << " t=" << Simulator::Now ().GetSeconds ());
              pkt = 0;
              return;
            }
          {
            VBHeader cvb; AquaSimHeader cash;
            pkt->RemoveHeader (cash); pkt->PeekHeader (cvb); pkt->AddHeader (cash);
            CustodyTag ct;
            ct.relay = AquaSimAddress::ConvertFrom (GetNetDevice ()->GetAddress ()).GetAsInt ();
            ct.prev = cvb.GetForwardAddr ().GetAsInt ();
            pkt->AddByteTag (ct);
          }
          MACprepare (pkt);
          MACsend (pkt, 0);
          UpdateSelfTrust (true);
        }
      else
        {
          pkt = 0;
        }
    }
}
 
}  // namespace ns3
 