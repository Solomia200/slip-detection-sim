"""Command line: slipsim list | show | run | doctor."""
import argparse
import sys

import yaml
from pydantic import ValidationError

from slipsim.config import hand_label, list_scenarios, load_scenario, scenario_to_yaml


def _print_summary(manifest, out):
    s = manifest["summary"]
    print(f"  предмет утримано       : {'так' if s['held'] else 'НІ'}")
    print(f"  піднято на             : {s['object_rise_mm']:.1f} мм")
    print(f"  зсув відносно руки     : {s['relative_shift_mm']:+.2f} мм")
    print(f"  такселів у контакті    : {s['touching_taxels']} із {s['total_taxels']}")
    if s["watch_start_s"] is not None:
        first, gross = s["first_taxel_slip_s"], s["gross_slip_s"]
        print(f"  перший таксель ковзає  : {'немає' if first is None else f'{first:.3f} с'}"
              + ("" if first is None else f"  (w = {s['w_at_first_taxel_slip']:.2f})"))
        print(f"  предмет поїхав         : {'немає' if gross is None else f'{gross:.3f} с'}"
              + ("" if gross is None else f"  (w = {s['w_at_gross_slip']:.2f})"))
        if s["lead_s"] is not None:
            print(f"  запас часу             : {s['lead_s']*1e3:.0f} мс")
    print(f"  швидкість              : {manifest['speed_x_realtime']:.2f}x реального часу")
    print(f"  результати             : {out}")


def cmd_list(_):
    for name, path in list_scenarios().items():
        raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        print(f"  {name:<16} {raw.get('description', '')}")


def cmd_show(args):
    print(scenario_to_yaml(load_scenario(args.scenario, args.set)), end="")


def cmd_run(args):
    from slipsim.episode import run_episode
    scenario = load_scenario(args.scenario, args.set)
    print(f"{scenario.name}: {hand_label(scenario)}, "
          f"{scenario.pad.rows}x{scenario.pad.rows} такселів, крок {scenario.pad.pitch_mm} мм")
    out, manifest = run_episode(scenario, view=args.view, out_root=args.out)
    _print_summary(manifest, out)


def cmd_doctor(args):
    from slipsim.episode import simulate
    failed = False
    for n in (3, 4, 6):
        scenario = load_scenario("grasp_lift", [f"hand.n_fingers={n}"])
        _, _, summary, wall = simulate(scenario)
        ok = summary["held"] and summary["object_rise_mm"] > 40
        failed = failed or not ok
        print(f"  [{'PASS' if ok else 'FAIL'}] {n} пальців: піднято {summary['object_rise_mm']:.1f} мм"
              f" ({wall:.1f} с)")
    return 1 if failed else 0


def main(argv=None):
    parser = argparse.ArgumentParser(prog="slipsim", description="Симуляція гріпера і ковзання")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("list", help="показати доступні сценарії").set_defaults(func=cmd_list)
    sub.add_parser("doctor", help="швидка перевірка, що все працює").set_defaults(func=cmd_doctor)
    for name, func, text in (("show", cmd_show, "показати всі параметри сценарію"),
                             ("run", cmd_run, "запустити сценарій")):
        cmd = sub.add_parser(name, help=text)
        cmd.add_argument("scenario", help="назва сценарію або шлях до .yaml")
        cmd.add_argument("--set", action="append", metavar="шлях=значення",
                         help="змінити параметр, напр. hand.n_fingers=6")
        cmd.set_defaults(func=func)
        if name == "run":
            cmd.add_argument("--view", action="store_true", help="показати 3D-вікно")
            cmd.add_argument("--out", help="папка для результатів (типово ./results)")

    args = parser.parse_args(argv)
    try:
        return args.func(args) or 0
    except ValidationError as error:
        print("Помилка в параметрах сценарію:", file=sys.stderr)
        for item in error.errors():
            where = ".".join(str(part) for part in item["loc"])
            print(f"  {where}: {item['msg']}", file=sys.stderr)
    except (ValueError, FileNotFoundError) as error:
        print(f"Помилка: {error}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
