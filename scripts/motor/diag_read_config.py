# scripts/diag_read_config.py
import berkeley_humanoid_lite_lowlevel.recoil as recoil

def p(name, v):
    print(f"{name:32s}= {v}")

def main():
    args = recoil.util.get_args()  # -c, -i 파싱
    bus = recoil.Bus(channel=args.channel, bitrate=1_000_000)
    dev = args.id

    poles    = bus.read_motor_pole_pairs(dev)
    phaseord = bus.read_motor_phase_order(dev)
    cpr      = bus.read_encoder_cpr(dev)
    gear     = bus.read_gear_ratio(dev)
    pos_off  = bus.read_position_offset(dev)
    flux_off = bus.read_encoder_flux_offset(dev)

    print("=== CONFIG ===")
    p("MOTOR_POLE_PAIRS", poles)
    p("MOTOR_PHASE_ORDER", phaseord)
    p("ENCODER_CPR", cpr)
    p("GEAR_RATIO", gear)
    p("POSITION_OFFSET(ctrl)", pos_off)
    p("ENCODER_FLUX_OFFSET", flux_off)

    bus.stop()

if __name__ == "__main__":
    main()
