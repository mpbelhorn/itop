"""
A module for analyzing mirror data.
"""

import numpy as np
from itop import N_HPFS, N_AIR

def plot_transmittance(data_sets, **kwargs):
  prism_id = kwargs.pop('prism_id', '')
  force_overwrite = kwargs.pop('overwrite', False)

  results = np.array([transmission(d, **kwargs) for d in data_sets])

  fig, axis = plt.subplots(1, 1, figsize=(10,5))
  number_of_trials = len(results)
  ordinates = range(1, number_of_trials + 1)
  plt.errorbar(
      ordinates,
      [i[0]*100. for i in results],
      yerr=[(i[1]+i[2])*100. for i in results],
      ls='None', marker='o')
  axis.yaxis.get_major_formatter().set_useOffset(False)
  plt.xlim(0, number_of_trials + 1)
  plt.ylim(99.5, 100)

  avg = np.mean(results[:,0]) * 100
  avg_error = (
      (sqrt(sum([i**2 for i in [j[1] + j[2] for j in results]])) * 100) /
       sqrt(len(results)))
  plt.axhline(avg, color='b', ls='--')
  axis.text(
      0.95, 0.03,
      'Avg = {:03.3f}$\pm${:03.3f}'.format(avg, avg_error),
      verticalalignment='bottom', horizontalalignment='right',
      transform=axis.transAxes, color='b', fontsize=15)
  plt.title('Prism ' + prism_id + ' Transmittance', fontsize=18)
  plt.ylabel('T [%/m]', fontsize=18)
  tick_labels = [str(i) for i in ordinates]
  tick_labels.insert(0, '')
  plt.xticks(range(number_of_trials + 1), tick_labels);
  plt.xlabel('Trial', fontsize=18)
  axis.yaxis.set_tick_params(labelsize=12)
  axis.xaxis.set_tick_params(labelsize=12)
  output_path = '/lab/data/mirrors/{}/transmittance_{}.pdf'.format(
      prism_id, prism_id)
  if not os.path.exists(output_path) or force_overwrite:
    print('Saving output')
    plt.savefig(output_path)
  else:
    print('Output not saved!')
  plt.show()
