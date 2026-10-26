# SPDX-License-Identifier: Apache-2.0
"""Optional genuine integration points for the three user-provided donor projects.

This module uses duck-typed HapticSight objects and optionally invokes the
installed SynapDrive-AI drift monitor. The core does NOT depend on donors.
"""
from __future__ import annotations
import math
from typing import Any


def from_hapticsight(force_torque_sample: Any, gravity_sensor_xyz: tuple[float, float, float],
                      now_s: float, max_age_ms: float=50.0) -> list[float]:
    """Translate ohip_interfaces.force_torque.ForceTorqueSample into CSV row.

    Important: HapticSight's timestamps are UTC seconds; match the clock domain.
    A single measurement is not sufficient to identify a 3-D CoM.
    """
    quality=force_torque_sample.quality
    health=str(getattr(quality.health, 'value', quality.health))
    if not quality.is_usable(now_utc_s=now_s):
        raise ValueError('unhealthy HapticSight force/torque sample')
    if quality.age_ms(now_utc_s=now_s)>max_age_ms:
        raise ValueError('stale HapticSight force/torque sample')
    if health not in ('NOMINAL','DEGRADED'):
        raise ValueError('unknown HapticSight health')
    if getattr(quality, 'frame', '') and quality.frame != force_torque_sample.frame:
        raise ValueError('sensor frames do not match')
    f=force_torque_sample.force
    t=force_torque_sample.torque
    vals=[float(x) for x in gravity_sensor_xyz]+[f.x,f.y,f.z,t.x,t.y,t.z]
    if len(vals)!=9 or not all(math.isfinite(float(x)) for x in vals):
        raise ValueError('nonfinite measurement')
    q=1.0 if health=='NOMINAL' else .5
    # Last 2 flags: equilibrium must be confirmed by independent contact/IMU
    # screening, so default to NOT STATIC. Caller must set it explicitly after
    # verifying a static interval. Never silently assert safe operating state.
    return [float(quality.sample_timestamp_utc_s),*vals,q,0.0,1.0]


def optional_synapdrive_drift(training_features: list[list[float]],
                              current_features: list[float]):
    """Use actual SynapDrive FeatureDriftMonitor when installed.

    Results are only a drift warning, not a safety certificate. Never learn from
    a known failing or untrusted period without separate validation.
    """
    try:
        from synapdrive_ai.neuro.drift import FeatureDriftMonitor
    except ImportError as exc:
        raise RuntimeError('install the original SynapDrive-AI package to use its drift monitor') from exc
    monitor=FeatureDriftMonitor(quantile=.99)
    monitor.fit(training_features)
    return monitor.evaluate(current_features)


def assurance_telemetry_contract(source: str, frame: str, timestamp_s: float,
                                 payload_status: str, information_condition: float) -> dict:
    """Minimal traceable telemetry to pass to an assurance-case adapter.

    No claim is made that this dictionary directly satisfies every version of
    the assurance runtime's typed policy schema.
    """
    if not source or not frame or not math.isfinite(timestamp_s):
        raise ValueError('invalid provenance')
    return {'source_kind':source,'frame':frame,'timestamp_s':timestamp_s,
            'payload_status':payload_status,'condition':information_condition,
            'validated_on_hardware':False}
