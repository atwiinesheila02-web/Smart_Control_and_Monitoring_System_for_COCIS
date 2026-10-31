#!/usr/bin/env python3
"""Add a button and simulated digital smoke alarm to a COPY of a Wokwi diagram."""
import argparse,json
from pathlib import Path

def add_wiring(data, board_id=None):
    parts=data.get('parts',[]);connections=data.get('connections',[])
    boards=[p for p in parts if 'esp32' in p.get('type','').lower() and 's3' in p.get('type','').lower()]
    if board_id is not None:boards=[p for p in boards if p.get('id')==board_id]
    if len(boards)!=1:raise ValueError('Expected one ESP32-S3 board; select it with --board-id if necessary.')
    board=boards[0];bid=board['id']
    if any(p.get('id') in {'emergency_btn','smoke_alarm_sim'} for p in parts):
        raise ValueError('Emergency parts already exist; inspect the diagram instead of adding duplicates.')
    busy={f'{bid}:21',f'{bid}:42',f'{bid}:GPIO21',f'{bid}:GPIO42'}
    if any(endpoint in busy for c in connections for endpoint in c[:2]):
        raise ValueError('GPIO21 or GPIO42 already has wiring. Review the pins before adding emergency inputs.')
    top=float(board.get('top',0));left=float(board.get('left',0))+300
    parts.extend([
        {'type':'wokwi-pushbutton','id':'emergency_btn','top':top-70,'left':left,'attrs':{'color':'red','label':'EMERGENCY EXIT'}},
        {'type':'wokwi-slide-switch','id':'smoke_alarm_sim','top':top+20,'left':left,'attrs':{'bounce':'0'}}
    ])
    connections.extend([
        ['emergency_btn:1.l',f'{bid}:21','orange',[]],
        ['emergency_btn:2.l',f'{bid}:GND.1','black',[]],
        ['smoke_alarm_sim:2',f'{bid}:42','purple',[]],
        ['smoke_alarm_sim:1',f'{bid}:GND.1','black',[]],
        ['smoke_alarm_sim:3',f'{bid}:3V3.1','red',[]]
    ])
    data['parts']=parts;data['connections']=connections
    return data

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('diagram',type=Path)
    parser.add_argument('--board-id')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    output=args.output or args.diagram.with_name('diagram_emergency.json')
    if output.resolve()==args.diagram.resolve():parser.error('Output must be a new file, not the original diagram.')
    if output.exists():parser.error('Output already exists; choose another --output path.')
    try:data=add_wiring(json.loads(args.diagram.read_text()),args.board_id)
    except (OSError,ValueError,TypeError,KeyError) as e:parser.error(str(e))
    output.write_text(json.dumps(data,indent=2)+'\n')
    print(f'Created {output}; original diagram preserved. Smoke switch LEFT=clear, RIGHT=alarm.')
if __name__=='__main__':main()
