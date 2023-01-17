import math
import yaml

def normalize_angle(phi):
  while (phi < 0):
    phi = phi + (math.pi * 2)
  
  while (phi > (math.pi * 2)):
    phi = phi - (math.pi * 2)
    
  return phi

def main(args=None):
  f = open('/home/ros/raven_ws/src/raven/line_finder/scripts/foo.yaml')
  data = yaml.load(f)
  segs = data['line_segments']
  number_lines = len(segs)
  for i in range(number_lines):
    a1 = segs[i]['angle']
    for j in range(number_lines - i - 1):
      a2 = segs[j + i + 1]['angle']
      diff = normalize_angle(a1 + a2)
      print(f"[{i}] angle[{i}]: {a1}, angle[{j + i + 1}]: {a2}, diff: {diff}")
  
if __name__ == '__main__':
    main()
