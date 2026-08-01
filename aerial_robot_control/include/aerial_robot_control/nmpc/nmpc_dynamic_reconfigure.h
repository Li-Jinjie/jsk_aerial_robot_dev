#pragma once

#include "aerial_robot_control/NMPCConfig.h"

#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace aerial_robot_control
{
namespace nmpc
{

using NMPCConfigMask = uint32_t;

namespace NMPCConfigFields
{
constexpr NMPCConfigMask FLAG = 1u << 0;
constexpr NMPCConfigMask QP_XY = 1u << 1;
constexpr NMPCConfigMask QP_Z = 1u << 2;
constexpr NMPCConfigMask QV_XY = 1u << 3;
constexpr NMPCConfigMask QV_Z = 1u << 4;
constexpr NMPCConfigMask QQ_XY = 1u << 5;
constexpr NMPCConfigMask QQ_Z = 1u << 6;
constexpr NMPCConfigMask QW_XY = 1u << 7;
constexpr NMPCConfigMask QW_Z = 1u << 8;
constexpr NMPCConfigMask QA = 1u << 9;
constexpr NMPCConfigMask RT = 1u << 10;
constexpr NMPCConfigMask RAC_D = 1u << 11;
constexpr NMPCConfigMask QT = 1u << 12;
constexpr NMPCConfigMask RTC_D = 1u << 13;
constexpr NMPCConfigMask PM_XY = 1u << 14;
constexpr NMPCConfigMask PM_Z = 1u << 15;
constexpr NMPCConfigMask OM_XY = 1u << 16;
constexpr NMPCConfigMask OM_Z = 1u << 17;
constexpr NMPCConfigMask ENLARGE_FACTOR = 1u << 18;

constexpr NMPCConfigMask ALL_PARAMETERS = QP_XY | QP_Z | QV_XY | QV_Z | QQ_XY | QQ_Z | QW_XY | QW_Z | QA | RT | RAC_D |
                                          QT | RTC_D | PM_XY | PM_Z | OM_XY | OM_Z | ENLARGE_FACTOR;
}  // namespace NMPCConfigFields

inline NMPCConfigMask getNMPCConfigChangeMask(const NMPCConfig& previous, const NMPCConfig& current)
{
  using namespace NMPCConfigFields;
  NMPCConfigMask mask = 0;
  if (previous.nmpc_flag != current.nmpc_flag)
    mask |= FLAG;
  if (previous.Qp_xy != current.Qp_xy)
    mask |= QP_XY;
  if (previous.Qp_z != current.Qp_z)
    mask |= QP_Z;
  if (previous.Qv_xy != current.Qv_xy)
    mask |= QV_XY;
  if (previous.Qv_z != current.Qv_z)
    mask |= QV_Z;
  if (previous.Qq_xy != current.Qq_xy)
    mask |= QQ_XY;
  if (previous.Qq_z != current.Qq_z)
    mask |= QQ_Z;
  if (previous.Qw_xy != current.Qw_xy)
    mask |= QW_XY;
  if (previous.Qw_z != current.Qw_z)
    mask |= QW_Z;
  if (previous.Qa != current.Qa)
    mask |= QA;
  if (previous.Rt != current.Rt)
    mask |= RT;
  if (previous.Rac_d != current.Rac_d)
    mask |= RAC_D;
  if (previous.Qt != current.Qt)
    mask |= QT;
  if (previous.Rtc_d != current.Rtc_d)
    mask |= RTC_D;
  if (previous.pMxy != current.pMxy)
    mask |= PM_XY;
  if (previous.pMz != current.pMz)
    mask |= PM_Z;
  if (previous.oMxy != current.oMxy)
    mask |= OM_XY;
  if (previous.oMz != current.oMz)
    mask |= OM_Z;
  if (previous.enlarge_factor != current.enlarge_factor)
    mask |= ENLARGE_FACTOR;
  return mask;
}

inline std::vector<std::string> getNMPCConfigFieldNames(NMPCConfigMask mask)
{
  using namespace NMPCConfigFields;
  const std::vector<std::pair<NMPCConfigMask, const char*>> fields = {
    { QP_XY, "Qp_xy" }, { QP_Z, "Qp_z" },   { QV_XY, "Qv_xy" },
    { QV_Z, "Qv_z" },   { QQ_XY, "Qq_xy" }, { QQ_Z, "Qq_z" },
    { QW_XY, "Qw_xy" }, { QW_Z, "Qw_z" },   { QA, "Qa" },
    { RT, "Rt" },       { RAC_D, "Rac_d" }, { QT, "Qt" },
    { RTC_D, "Rtc_d" }, { PM_XY, "pMxy" },  { PM_Z, "pMz" },
    { OM_XY, "oMxy" },  { OM_Z, "oMz" },    { ENLARGE_FACTOR, "enlarge_factor" },
  };

  std::vector<std::string> names;
  for (const auto& field : fields)
  {
    if (mask & field.first)
      names.emplace_back(field.second);
  }
  return names;
}

inline std::string formatNMPCConfigFieldValues(const NMPCConfig& config, NMPCConfigMask mask)
{
  using namespace NMPCConfigFields;
  std::ostringstream values;
  bool is_first = true;
  const auto append = [&](NMPCConfigMask field, const char* name, double value) {
    if (!(mask & field))
      return;
    if (!is_first)
      values << ", ";
    values << name << "=" << value;
    is_first = false;
  };

  append(QP_XY, "Qp_xy", config.Qp_xy);
  append(QP_Z, "Qp_z", config.Qp_z);
  append(QV_XY, "Qv_xy", config.Qv_xy);
  append(QV_Z, "Qv_z", config.Qv_z);
  append(QQ_XY, "Qq_xy", config.Qq_xy);
  append(QQ_Z, "Qq_z", config.Qq_z);
  append(QW_XY, "Qw_xy", config.Qw_xy);
  append(QW_Z, "Qw_z", config.Qw_z);
  append(QA, "Qa", config.Qa);
  append(RT, "Rt", config.Rt);
  append(RAC_D, "Rac_d", config.Rac_d);
  append(QT, "Qt", config.Qt);
  append(RTC_D, "Rtc_d", config.Rtc_d);
  append(PM_XY, "pMxy", config.pMxy);
  append(PM_Z, "pMz", config.pMz);
  append(OM_XY, "oMxy", config.oMxy);
  append(OM_Z, "oMz", config.oMz);
  append(ENLARGE_FACTOR, "enlarge_factor", config.enlarge_factor);
  return values.str();
}

class NMPCConfigUpdateState
{
public:
  void ingest(const NMPCConfig& config, NMPCConfigMask supported_mask)
  {
    if (!has_received_config_)
    {
      has_received_config_ = true;
      last_received_config_ = config;
      if (config.nmpc_flag)
      {
        pending_config_ = config;
        pending_mask_ |= supported_mask;
      }
      return;
    }

    const bool was_enabled = last_received_config_.nmpc_flag;
    const NMPCConfigMask change_mask = getNMPCConfigChangeMask(last_received_config_, config);
    last_received_config_ = config;

    if (!config.nmpc_flag)
    {
      pending_mask_ = 0;
      return;
    }

    pending_config_ = config;
    pending_mask_ |= was_enabled ? (change_mask & NMPCConfigFields::ALL_PARAMETERS) : supported_mask;
  }

  bool takePending(NMPCConfig& config, NMPCConfigMask& mask)
  {
    if (pending_mask_ == 0)
      return false;

    config = pending_config_;
    mask = pending_mask_;
    pending_mask_ = 0;
    return true;
  }

private:
  bool has_received_config_ = false;
  NMPCConfig last_received_config_;
  NMPCConfig pending_config_;
  NMPCConfigMask pending_mask_ = 0;
};

}  // namespace nmpc
}  // namespace aerial_robot_control
