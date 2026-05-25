from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Union


EARTH_RADIUS_M = 6378137.0
VALID_HEADING_TYPES = {"NARROW_INT", "NARROW_FLOAT", "WIDE_INT", "WIDE_FLOAT"}
VALID_INS_STATUSES = {"INS_ALIGNMENT_COMPLETE", "INS_SOLUTION_GOOD"}
VALID_INS_POSITION_TYPES = {"INS_RTKFIXED", "INS_RTKFLOAT", "INS_PSRDIFF", "INS_FIXEDPOS"}


class BynavParseError(ValueError):
    """Raised when a Bynav/NMEA line cannot be parsed."""


@dataclass(frozen=True)
class HeaderMeta:
    message_name: str
    port: str | None = None
    gps_week: int | None = None
    gps_seconds: float | None = None


@dataclass(frozen=True)
class GpggaRecord:
    utc_time: str
    latitude_deg: float
    longitude_deg: float
    fix_quality: int
    satellite_count: int
    hdop: float | None
    altitude_msl_m: float | None
    geoid_separation_m: float | None


@dataclass(frozen=True)
class BestPosRecord:
    header: HeaderMeta
    solution_status: str
    position_type: str
    latitude_deg: float
    longitude_deg: float
    height_msl_m: float
    undulation_m: float
    datum: str
    lat_sigma_m: float | None
    lon_sigma_m: float | None
    height_sigma_m: float | None
    differential_age_sec: float | None
    solution_age_sec: float | None


@dataclass(frozen=True)
class Heading2Record:
    header: HeaderMeta
    solution_status: str
    heading_type: str
    baseline_length_m: float
    heading_deg: float
    pitch_deg: float
    heading_sigma_deg: float | None
    pitch_sigma_deg: float | None


@dataclass(frozen=True)
class InspvaxRecord:
    header: HeaderMeta
    ins_status: str
    position_type: str
    latitude_deg: float
    longitude_deg: float
    height_msl_m: float
    undulation_m: float
    north_velocity_mps: float
    east_velocity_mps: float
    up_velocity_mps: float
    roll_deg: float
    pitch_deg: float
    azimuth_deg: float
    lat_sigma_m: float | None = None
    lon_sigma_m: float | None = None
    height_sigma_m: float | None = None
    north_velocity_sigma_mps: float | None = None
    east_velocity_sigma_mps: float | None = None
    up_velocity_sigma_mps: float | None = None
    roll_sigma_deg: float | None = None
    pitch_sigma_deg: float | None = None
    azimuth_sigma_deg: float | None = None
    extended_status: str | None = None
    time_since_update_sec: float | None = None


@dataclass(frozen=True)
class InsCalStatusRecord:
    header: HeaderMeta
    calibration_type: str
    rbv_x_deg: float
    rbv_y_deg: float
    rbv_z_deg: float
    rbv_x_uncertainty_deg: float | None
    rbv_y_uncertainty_deg: float | None
    rbv_z_uncertainty_deg: float | None
    source_status: str
    calibration_count: int | None


ParsedRecord = Union[GpggaRecord, BestPosRecord, Heading2Record, InspvaxRecord, InsCalStatusRecord]


@dataclass(frozen=True)
class VehiclePoseSample:
    gps_week: int | None
    gps_seconds: float | None
    latitude_deg: float | None
    longitude_deg: float | None
    height_msl_m: float | None
    x_east_m: float | None
    y_north_m: float | None
    north_velocity_mps: float | None
    east_velocity_mps: float | None
    up_velocity_mps: float | None
    roll_deg: float | None
    pitch_deg: float | None
    azimuth_deg: float | None
    yaw_rad: float | None
    heading_deg: float | None
    position_source: str | None
    heading_source: str | None
    position_valid: bool
    heading_valid: bool
    attitude_valid: bool
    ins_solution_good: bool
    ins_alignment_complete: bool
    position_type: str | None = None
    heading_type: str | None = None
    ins_status: str | None = None
    heading_sigma_deg: float | None = None
    lat_sigma_m: float | None = None
    lon_sigma_m: float | None = None
    height_sigma_m: float | None = None
    north_velocity_sigma_mps: float | None = None
    east_velocity_sigma_mps: float | None = None
    up_velocity_sigma_mps: float | None = None
    roll_sigma_deg: float | None = None
    pitch_sigma_deg: float | None = None
    azimuth_sigma_deg: float | None = None
    rbv_status: str | None = None
    rbv_uncertainty_deg: tuple[float | None, float | None, float | None] | None = None


class BynavAsciiParser:
    """Parser for a small subset of Bynav ASCII/NMEA logs used in this project."""

    def parse_line(self, line: str) -> ParsedRecord:
        text = line.strip()
        if not text:
            raise BynavParseError("empty line")
        if text.startswith("$GPGGA"):
            return self._parse_gpgga(text)
        if text.startswith("#BESTPOSA"):
            return self._parse_bestposa(text)
        if text.startswith("#HEADING2A"):
            return self._parse_heading2a(text)
        if text.startswith("#INSPVAXA"):
            return self._parse_inspvaxa(text)
        if text.startswith("#INSCALSTATUSA"):
            return self._parse_inscalstatusa(text)
        raise BynavParseError(f"unsupported line prefix: {text[:16]}")

    def _parse_gpgga(self, line: str) -> GpggaRecord:
        payload = self._strip_crc(line)
        fields = payload.split(",")
        if len(fields) < 12:
            raise BynavParseError("GPGGA has too few fields")
        return GpggaRecord(
            utc_time=fields[1],
            latitude_deg=_parse_nmea_latlon(fields[2], fields[3]),
            longitude_deg=_parse_nmea_latlon(fields[4], fields[5]),
            fix_quality=_to_int(fields[6]),
            satellite_count=_to_int(fields[7]),
            hdop=_to_float_or_none(fields[8]),
            altitude_msl_m=_to_float_or_none(fields[9]),
            geoid_separation_m=_to_float_or_none(fields[11]),
        )

    def _parse_bestposa(self, line: str) -> BestPosRecord:
        header, body = self._split_ascii(line)
        if len(body) < 15:
            raise BynavParseError("BESTPOSA body has too few fields")
        return BestPosRecord(
            header=header,
            solution_status=body[0],
            position_type=body[1],
            latitude_deg=_to_float(body[2]),
            longitude_deg=_to_float(body[3]),
            height_msl_m=_to_float(body[4]),
            undulation_m=_to_float(body[5]),
            datum=body[6],
            lat_sigma_m=_to_float_or_none(body[7]),
            lon_sigma_m=_to_float_or_none(body[8]),
            height_sigma_m=_to_float_or_none(body[9]),
            differential_age_sec=_to_float_or_none(body[11]),
            solution_age_sec=_to_float_or_none(body[12]),
        )

    def _parse_heading2a(self, line: str) -> Heading2Record:
        header, body = self._split_ascii(line)
        if len(body) < 8:
            raise BynavParseError("HEADING2A body has too few fields")
        return Heading2Record(
            header=header,
            solution_status=body[0],
            heading_type=body[1],
            baseline_length_m=_to_float(body[2]),
            heading_deg=_to_float(body[3]),
            pitch_deg=_to_float(body[4]),
            heading_sigma_deg=_to_float_or_none(body[6]),
            pitch_sigma_deg=_to_float_or_none(body[7]),
        )

    def _parse_inspvaxa(self, line: str) -> InspvaxRecord:
        header, body = self._split_ascii(line)
        if len(body) < 12:
            raise BynavParseError("INSPVAXA body has too few fields")
        return InspvaxRecord(
            header=header,
            ins_status=body[0],
            position_type=body[1].strip(),
            latitude_deg=_to_float(body[2]),
            longitude_deg=_to_float(body[3]),
            height_msl_m=_to_float(body[4]),
            undulation_m=_to_float(body[5]),
            north_velocity_mps=_to_float(body[6]),
            east_velocity_mps=_to_float(body[7]),
            up_velocity_mps=_to_float(body[8]),
            roll_deg=_to_float(body[9]),
            pitch_deg=_to_float(body[10]),
            azimuth_deg=_to_float(body[11]),
            lat_sigma_m=_field_float_or_none(body, 12),
            lon_sigma_m=_field_float_or_none(body, 13),
            height_sigma_m=_field_float_or_none(body, 14),
            north_velocity_sigma_mps=_field_float_or_none(body, 15),
            east_velocity_sigma_mps=_field_float_or_none(body, 16),
            up_velocity_sigma_mps=_field_float_or_none(body, 17),
            roll_sigma_deg=_field_float_or_none(body, 18),
            pitch_sigma_deg=_field_float_or_none(body, 19),
            azimuth_sigma_deg=_field_float_or_none(body, 20),
            extended_status=body[21] if len(body) > 21 else None,
            time_since_update_sec=_field_float_or_none(body, 22),
        )

    def _parse_inscalstatusa(self, line: str) -> InsCalStatusRecord:
        header, body = self._split_ascii(line)
        if len(body) < 9:
            raise BynavParseError("INSCALSTATUSA body has too few fields")
        return InsCalStatusRecord(
            header=header,
            calibration_type=body[0],
            rbv_x_deg=_to_float(body[1]),
            rbv_y_deg=_to_float(body[2]),
            rbv_z_deg=_to_float(body[3]),
            rbv_x_uncertainty_deg=_to_float_or_none(body[4]),
            rbv_y_uncertainty_deg=_to_float_or_none(body[5]),
            rbv_z_uncertainty_deg=_to_float_or_none(body[6]),
            source_status=body[7],
            calibration_count=_to_int_or_none(body[8]),
        )

    def _split_ascii(self, line: str) -> tuple[HeaderMeta, list[str]]:
        payload = self._strip_crc(line)
        if ";" not in payload:
            raise BynavParseError("ASCII log missing ';' separator")
        header_raw, body_raw = payload.split(";", 1)
        header_fields = header_raw.lstrip("#").split(",")
        body_fields = body_raw.split(",")
        header = HeaderMeta(
            message_name=header_fields[0],
            port=header_fields[1] if len(header_fields) > 1 else None,
            gps_week=_to_int_or_none(header_fields[5]) if len(header_fields) > 5 else None,
            gps_seconds=_to_float_or_none(header_fields[6]) if len(header_fields) > 6 else None,
        )
        return header, body_fields

    @staticmethod
    def _strip_crc(line: str) -> str:
        return line.split("*", 1)[0].strip()


class VehiclePoseEstimator:
    """Builds vehicle pose samples from parsed Bynav logs.

    Current strategy:
    - position: BESTPOSA first, INSPVAXA only when INS status is usable
    - attitude/velocity: INSPVAXA as soon as it is non-zero, even while INS is aligning
    - heading: INSPVAXA azimuth first when present, otherwise HEADING2A if sigma is small
    """

    def __init__(self, heading_sigma_threshold_deg: float = 5.0) -> None:
        self.heading_sigma_threshold_deg = heading_sigma_threshold_deg
        self.origin_lat_deg: float | None = None
        self.origin_lon_deg: float | None = None
        self.last_bestpos: BestPosRecord | None = None
        self.last_heading2: Heading2Record | None = None
        self.last_inspvax: InspvaxRecord | None = None
        self.last_inscalstatus: InsCalStatusRecord | None = None

    def update(self, record: ParsedRecord) -> VehiclePoseSample | None:
        if isinstance(record, BestPosRecord):
            self.last_bestpos = record
        elif isinstance(record, Heading2Record):
            self.last_heading2 = record
        elif isinstance(record, InspvaxRecord):
            self.last_inspvax = record
        elif isinstance(record, InsCalStatusRecord):
            self.last_inscalstatus = record
        elif isinstance(record, GpggaRecord):
            return None
        return self.current_pose_sample()

    def current_pose_sample(self) -> VehiclePoseSample | None:
        position_lat = None
        position_lon = None
        position_h = None
        position_source = None
        position_type = None
        gps_week = None
        gps_seconds = None
        ins_status = None
        lat_sigma_m = None
        lon_sigma_m = None
        height_sigma_m = None

        if self._inspvax_position_good(self.last_inspvax):
            inspvax = self.last_inspvax
            assert inspvax is not None
            position_lat = inspvax.latitude_deg
            position_lon = inspvax.longitude_deg
            position_h = inspvax.height_msl_m
            position_source = "INSPVAXA"
            position_type = inspvax.position_type
            gps_week = inspvax.header.gps_week
            gps_seconds = inspvax.header.gps_seconds
            ins_status = inspvax.ins_status
            lat_sigma_m = inspvax.lat_sigma_m
            lon_sigma_m = inspvax.lon_sigma_m
            height_sigma_m = inspvax.height_sigma_m
        elif self.last_bestpos is not None:
            bestpos = self.last_bestpos
            position_lat = bestpos.latitude_deg
            position_lon = bestpos.longitude_deg
            position_h = bestpos.height_msl_m
            position_source = "BESTPOSA"
            position_type = bestpos.position_type
            gps_week = bestpos.header.gps_week
            gps_seconds = bestpos.header.gps_seconds
            lat_sigma_m = bestpos.lat_sigma_m
            lon_sigma_m = bestpos.lon_sigma_m
            height_sigma_m = bestpos.height_sigma_m
        elif self._inspvax_position_present(self.last_inspvax):
            inspvax = self.last_inspvax
            assert inspvax is not None
            position_lat = inspvax.latitude_deg
            position_lon = inspvax.longitude_deg
            position_h = inspvax.height_msl_m
            position_source = "INSPVAXA_ALIGNING"
            position_type = inspvax.position_type
            gps_week = inspvax.header.gps_week
            gps_seconds = inspvax.header.gps_seconds
            ins_status = inspvax.ins_status
            lat_sigma_m = inspvax.lat_sigma_m
            lon_sigma_m = inspvax.lon_sigma_m
            height_sigma_m = inspvax.height_sigma_m

        x_east = None
        y_north = None
        position_valid = position_lat is not None and position_lon is not None
        if position_valid:
            x_east, y_north = self._to_local_xy(position_lat, position_lon)

        yaw_rad = None
        heading_deg = None
        heading_source = None
        heading_type = None
        heading_sigma_deg = None
        heading_valid = False
        north_velocity_mps = None
        east_velocity_mps = None
        up_velocity_mps = None
        roll_deg = None
        pitch_deg = None
        azimuth_deg = None
        north_velocity_sigma_mps = None
        east_velocity_sigma_mps = None
        up_velocity_sigma_mps = None
        roll_sigma_deg = None
        pitch_sigma_deg = None
        azimuth_sigma_deg = None
        attitude_valid = False
        ins_solution_good = False
        ins_alignment_complete = False

        if self._inspvax_attitude_present(self.last_inspvax):
            inspvax = self.last_inspvax
            assert inspvax is not None
            north_velocity_mps = inspvax.north_velocity_mps
            east_velocity_mps = inspvax.east_velocity_mps
            up_velocity_mps = inspvax.up_velocity_mps
            roll_deg = inspvax.roll_deg
            pitch_deg = inspvax.pitch_deg
            azimuth_deg = inspvax.azimuth_deg
            north_velocity_sigma_mps = inspvax.north_velocity_sigma_mps
            east_velocity_sigma_mps = inspvax.east_velocity_sigma_mps
            up_velocity_sigma_mps = inspvax.up_velocity_sigma_mps
            roll_sigma_deg = inspvax.roll_sigma_deg
            pitch_sigma_deg = inspvax.pitch_sigma_deg
            azimuth_sigma_deg = inspvax.azimuth_sigma_deg
            heading_deg = inspvax.azimuth_deg
            yaw_rad = heading_deg_to_yaw_rad(heading_deg)
            heading_source = "INSPVAXA"
            heading_type = inspvax.position_type
            ins_status = inspvax.ins_status
            heading_sigma_deg = inspvax.azimuth_sigma_deg
            heading_valid = True
            attitude_valid = True
            ins_solution_good = inspvax.ins_status == "INS_SOLUTION_GOOD"
            ins_alignment_complete = inspvax.ins_status in VALID_INS_STATUSES
        elif self._heading2_valid(self.last_heading2):
            heading2 = self.last_heading2
            assert heading2 is not None
            heading_deg = heading2.heading_deg
            yaw_rad = heading_deg_to_yaw_rad(heading_deg)
            heading_source = "HEADING2A"
            heading_type = heading2.heading_type
            heading_sigma_deg = heading2.heading_sigma_deg
            heading_valid = True

        if not position_valid and not heading_valid:
            return None

        rbv_status = None
        rbv_uncertainty_deg = None
        if self.last_inscalstatus is not None:
            rbv_status = self.last_inscalstatus.source_status
            rbv_uncertainty_deg = (
                self.last_inscalstatus.rbv_x_uncertainty_deg,
                self.last_inscalstatus.rbv_y_uncertainty_deg,
                self.last_inscalstatus.rbv_z_uncertainty_deg,
            )

        return VehiclePoseSample(
            gps_week=gps_week,
            gps_seconds=gps_seconds,
            latitude_deg=position_lat,
            longitude_deg=position_lon,
            height_msl_m=position_h,
            x_east_m=x_east,
            y_north_m=y_north,
            north_velocity_mps=north_velocity_mps,
            east_velocity_mps=east_velocity_mps,
            up_velocity_mps=up_velocity_mps,
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            azimuth_deg=azimuth_deg,
            yaw_rad=yaw_rad,
            heading_deg=heading_deg,
            position_source=position_source,
            heading_source=heading_source,
            position_valid=position_valid,
            heading_valid=heading_valid,
            attitude_valid=attitude_valid,
            ins_solution_good=ins_solution_good,
            ins_alignment_complete=ins_alignment_complete,
            position_type=position_type,
            heading_type=heading_type,
            ins_status=ins_status,
            heading_sigma_deg=heading_sigma_deg,
            lat_sigma_m=lat_sigma_m,
            lon_sigma_m=lon_sigma_m,
            height_sigma_m=height_sigma_m,
            north_velocity_sigma_mps=north_velocity_sigma_mps,
            east_velocity_sigma_mps=east_velocity_sigma_mps,
            up_velocity_sigma_mps=up_velocity_sigma_mps,
            roll_sigma_deg=roll_sigma_deg,
            pitch_sigma_deg=pitch_sigma_deg,
            azimuth_sigma_deg=azimuth_sigma_deg,
            rbv_status=rbv_status,
            rbv_uncertainty_deg=rbv_uncertainty_deg,
        )

    def _to_local_xy(self, lat_deg: float, lon_deg: float) -> tuple[float, float]:
        if self.origin_lat_deg is None or self.origin_lon_deg is None:
            self.origin_lat_deg = lat_deg
            self.origin_lon_deg = lon_deg
            return 0.0, 0.0
        dlat = math.radians(lat_deg - self.origin_lat_deg)
        dlon = math.radians(lon_deg - self.origin_lon_deg)
        x_east = EARTH_RADIUS_M * dlon * math.cos(math.radians(self.origin_lat_deg))
        y_north = EARTH_RADIUS_M * dlat
        return x_east, y_north

    def _heading2_valid(self, record: Heading2Record | None) -> bool:
        if record is None:
            return False
        if record.heading_type not in VALID_HEADING_TYPES:
            return False
        if record.heading_sigma_deg is None:
            return False
        return record.heading_sigma_deg <= self.heading_sigma_threshold_deg

    @staticmethod
    def _inspvax_position_present(record: InspvaxRecord | None) -> bool:
        if record is None:
            return False
        if record.position_type not in VALID_INS_POSITION_TYPES:
            return False
        return not (record.latitude_deg == 0.0 and record.longitude_deg == 0.0)

    @staticmethod
    def _inspvax_position_good(record: InspvaxRecord | None) -> bool:
        if record is None:
            return False
        if record.ins_status not in VALID_INS_STATUSES:
            return False
        return VehiclePoseEstimator._inspvax_position_present(record)

    @staticmethod
    def _inspvax_attitude_present(record: InspvaxRecord | None) -> bool:
        if record is None:
            return False
        if not VehiclePoseEstimator._inspvax_position_present(record):
            return False
        return not (
            record.roll_deg == 0.0
            and record.pitch_deg == 0.0
            and record.azimuth_deg == 0.0
        )


def inspvax_has_final_solution(record: InspvaxRecord) -> bool:
    return record.ins_status in VALID_INS_STATUSES and record.position_type == "INS_RTKFIXED"


def inscalstatus_rbv_converged(
    record: InsCalStatusRecord,
    uncertainty_threshold_deg: float = 1.0,
) -> bool:
    uncertainties = [
        record.rbv_x_uncertainty_deg,
        record.rbv_y_uncertainty_deg,
        record.rbv_z_uncertainty_deg,
    ]
    if any(value is None for value in uncertainties):
        return False
    if record.source_status not in {"INS_ALIGNMENT_COMPLETE", "INS_SOLUTION_GOOD"}:
        return False
    return all(value <= uncertainty_threshold_deg for value in uncertainties if value is not None)


def inspvax_speed_mps(record: InspvaxRecord) -> float:
    return math.sqrt(
        (record.north_velocity_mps * record.north_velocity_mps)
        + (record.east_velocity_mps * record.east_velocity_mps)
        + (record.up_velocity_mps * record.up_velocity_mps)
    )


def heading_deg_to_yaw_rad(heading_deg: float) -> float:
    """Convert true-north clockwise heading into east-zero CCW yaw."""
    return wrap_to_pi((math.pi / 2.0) - math.radians(heading_deg))


def wrap_to_pi(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _parse_nmea_latlon(value: str, hemisphere: str) -> float:
    if not value or not hemisphere:
        raise BynavParseError("missing NMEA lat/lon field")
    raw = float(value)
    degrees = int(raw // 100)
    minutes = raw - (degrees * 100)
    decimal = degrees + (minutes / 60.0)
    if hemisphere in {"S", "W"}:
        decimal *= -1.0
    return decimal


def _to_float(value: str) -> float:
    try:
        return float(value)
    except ValueError as exc:
        raise BynavParseError(f"invalid float: {value}") from exc


def _to_float_or_none(value: str) -> float | None:
    if value == "":
        return None
    return _to_float(value)


def _field_float_or_none(fields: list[str], index: int) -> float | None:
    if index >= len(fields):
        return None
    return _to_float_or_none(fields[index])


def _to_int(value: str) -> int:
    try:
        return int(value)
    except ValueError as exc:
        raise BynavParseError(f"invalid int: {value}") from exc


def _to_int_or_none(value: str) -> int | None:
    if value == "":
        return None
    return _to_int(value)
