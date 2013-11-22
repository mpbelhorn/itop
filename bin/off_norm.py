def off_normal_power(off_normal_range, samples, normal_angle):
  data = []
  for i in np.arange(
      -off_normal_range,
      off_normal_range,
      2*off_normal_range / samples):
    r.position(math.degrees(i) + normal_angle, wait=True)
    time.sleep(0.3)
    t.center_beam()
    time.sleep(0.3)
    data.append((r.position(), p.average_power(), t.devices['monitor'].read()))
    print data[-1]
  da = np.array(data)
  output = np.array((np.array([i.value for i in da[:,0]]), da[:,1]/da[:,2], da[:,2])).T
  itop.utilities.save_object(output, './itop/data/off_normal_power.gz')
  return output

