#!/usr/bin/env python3
"""Add simulated entry/exit passage beams to a COPY of an ESP32-S3 Wokwi diagram."""
import argparse,json
from pathlib import Path

def add_wiring(data,board_id=None):
    parts=data.get('parts',[]);connections=data.get('connections',[])
    boards=[p for p in parts if 'esp32' in p.get('type','').lower() and 's3' in p.get('type','').lower()]
    if board_id is not None:boards=[p for p in boards if p.get('id')==board_id]
    if len(boards)!=1:raise ValueError('Expected one ESP32-S3 board; use --board-id if necessary.')
    board=boards[0];bid=board['id']
    if any(p.get('id') in {'passage_entry_beam','passage_exit_beam'} for p in parts):
        raise ValueError('Passage parts already exist; do not add duplicates.')
    busy={f'{bid}:19',f'{bid}:20',f'{bid}:GPIO19',f'{bid}:GPIO20'}
    if any(endpoint in busy for c in connections for endpoint in c[:2]):
        raise ValueError('GPIO19 or GPIO20 already has wiring; review pin allocation first.')
    top=float(board.get('top',0))+120;left=float(board.get('left',0))+300
    for offset,(part,pin,label) in enumerate([('passage_entry_beam',19,'ENTRY BEAM'),('passage_exit_beam',20,'EXIT BEAM')]):
        parts.append({'type':'wokwi-pushbutton','id':part,'top':top+offset*90,'left':left,
            'attrs':{'color':'blue','label':label,'bounce':'0'}})
        connections.extend([[f'{part}:1.l',f'{bid}:{pin}','blue',[]],[f'{part}:2.l',f'{bid}:GND.1','black',[]]])
    data['parts']=parts;data['connections']=connections
    return data

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('diagram',type=Path);parser.add_argument('--board-id');parser.add_argument('--output',type=Path)
    args=parser.parse_args();output=args.output or args.diagram.with_name('diagram_passage.json')
    if output.resolve()==args.diagram.resolve():parser.error('Output must be a new file, not the original.')
    if output.exists():parser.error('Output already exists; choose another --output path.')
    try:data=add_wiring(json.loads(args.diagram.read_text()),args.board_id)
    except (OSError,ValueError,TypeError,KeyError) as e:parser.error(str(e))
    output.write_text(json.dumps(data,indent=2)+'\n')
    print(f'Created {output}; original preserved. Hold/release a blue button to simulate passage.')
    print('GPIO19/20 are USB data pins on ESP32-S3: this prototype uses UART0, not native USB.')
if __name__=='__main__':main()
