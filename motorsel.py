#!/usr/local/bin/python
from __future__ import division
from utils import *
from units import *
from motor_utils import *


def calc(name, points, gear_ratio=1, motor_weight=0, wheel_diam=18, inertia=None, nmotors=2):
    # all torques in N-m. Speed in rpm, eff_radius in meters
    # inertia, of the motor only, in kg-m^2

    # wheel is 18" dia, so 0.228 m radius
    eff_radius = 0.228*(wheel_diam/18.0) / gear_ratio

    print "%d of %s at %g:1 to %g inch wheel" % (nmotors, name, gear_ratio, wheel_diam),
    print "  weight = %s" % weight_str(nmotors * motor_weight)

    for rpm, peak_torque, cont_torque in points:

	omega=rpm * 6.28 / 60
        speed = omega * eff_radius

        peak_force = nmotors * peak_torque / eff_radius
        peak_power = peak_force * speed
        cont_force = nmotors * cont_torque / eff_radius
        cont_power = cont_force * speed
        
        print "  speed=%s" % speed_str(speed)
        print "    peak force = %s peak power = %s" % (force_str(peak_force), power_str(peak_power))
        print "    cont force = %s cont power = %s" % (force_str(cont_force), power_str(cont_power))
        print "    gearhead peak torque = %s" % torque_str(peak_torque * gear_ratio)
	
	if inertia:
	    print "    accel time = %0.3f" % (omega / (peak_torque / inertia))

#calc('pmb31d', (6.26, 2.11, 0), (1.62, 1.62, 6000), 20, 2.5)

#calc('pmb32e', (11.40, 3.84, 0), (2.77, 2.77, 4450), 14, 4.4)

if 0:
    calc('GM90 double 160v 10:1',
         ((0, 83.4, 27.8),
          (270, 83.4, 27),
          (330, 28, 28)),
          1, 7.4+1.9, 16, 0.00094*100)
    # GM090-D1C1F

if 1:
    calc('GM115 double 320v 7:1 with bda3420d',
         ((0, 130, 42),
          (390, 130, 42)),
         1, 8.4+1.9, 18)
    calc('GM115 double 320v 5:1 with bda3420d',
         ((0, 92, 31),
          (520, 92, 31)),
         1, 8.4+1.9, 15)

# GM115-D1C1F
# http://www.baysidemotion.com/Web/ProdCenter.nsf/0d5bad4a449ecb108525695800610df9/b466c448e7d95e9285256c9800638b70?OpenDocument
# 600 lb radial load 1 inch from mounting surface.
#  prx = (3.5in) / (2.48 + X)
#  so 250 lb radial load 6 inches out. Plenty
# 200 lb axial load

# Use BDA3420DE. weight=1.9 kg
# Distributor: www.appliedmechatronics.com
# peak current about 25 A
# http://www.baysidemotion.com/Web/ProdCenter.nsf/0d5bad4a449ecb108525695800610df9/e53bdbddc608b2718525697b006c0dcf?OpenDocument


#calc('BE344L + G10', 14.4, 4.80, 3.19, 5000, 10, 5.3 + 2.18, 12)


#calc('Baldor BSM80N-333AA', 17.2, 4.3, 3.5, 7000, 20, 6.0)
if 0:
    calc('Baldor BSM80N-233AA', 
            ((0, 12.8, 3.2), 
             (7000, 2.6, 2.6)),
            24, 4.6, 1.717/100000)
# http://www.servosystems.com/baldor_n_series.htm

calc('Animatics SM3450', [(3400, 5.30, 1.77)], gear_ratio=10, motor_weight=2.9, wheel_diam=14)

#calc('SGMPH-15A', 14, 5, 5, 3000, 12, 0)



# SimpleServo
#   motor is 526-XX-3422NX1 (160v) at $820
#   gearbox is 518-52-010 at $835
#   drive is SS612A-3P (12-24 A) at $950
if 0: calc('SimpleServo 526-XX-3422NX1 with SSX12',
           ((0, 10.2, 3.0),
            (4000, 8.9, 2.3)),
           10, 0, 12)

if 1: calc('SimpleServo 520-25-320 ($1265) with 518-53-007 ($1190) and SS612A-3p()',
           ((0, conv_inlb_nm(min(3.8*24,220)), conv_inlb_nm(44)),
            (6000, conv_inlb_nm(min(3.8*24,220)), conv_inlb_nm(44))),
           7, conv_lbm_kg(15+5), 14)

if 0:
    calc('ecycle MG48',
         ((0, 0.48*60, 0.48*29),
          ((52-50*0.29)/0.051, 0.48*50, 0.48*29)),   # 52 volts, 50 amps
         66.0/22, 5.7, 18)
     
if 1:
    calc('sdp-si d50r10-067 + s9134a-pg010 ',
         ((0, conv_inoz_nm(825), conv_inoz_nm(165)),
          (5000, conv_inoz_nm(825), conv_inoz_nm(165))),
         10 * 1.0, conv_lbm_kg(8.6+7), 12,
         conv_inoz_per_sec2_kgm2(0.05))
    # torque constant = 21.6 oz-in / A
    # voltage constant = 16 V / krpm
    # resistance = 0.6 ohm
    # peak current = 38 A
    # voltage at 5000 rpm is 80 v
    # with peak current, IR drop is 23v
    # need 103 v drive voltage
    # want 110 v bus voltage minumum
    # 100 cells in series (1.1 v output)

if 0:
    calc('Schlenker 714303',
         ((0, 58.0, 21.6),
          (3000, 58.0, 21.6)),
         1, 8, 15)

calc('NPC T-74',
     ((0, conv_inlb_nm(1480), conv_inlb_nm(300)),
      (305, conv_inlb_nm(1480), conv_inlb_nm(300))),
     1, 6.5, 14)

# dyno says 300 in-lb at 43.1 amps
kt=conv_inlb_nm(300/43.1)
calc('NPC T-64, just thinking about electronics',
     ((0, kt*150, kt*75),
      (305, kt*100, kt*75)),
     1, 6.5, 20)

# dyno says 833 in-lb at 107.5 amps
kt=conv_inlb_nm(833/107.5)
calc('NPC T-74, just thinking about electronics',
     ((0, kt*150, kt*75),
      (305, kt*100, kt*75)),
     1, 6.5, 20)

calc('BS Etek',
     ((0, conv_inlb_nm(330*1.14), conv_inlb_nm(230*1.14)),
      (3000, conv_inlb_nm(330*1.14), conv_inlb_nm(160*1.14))),
      4, conv_lbm_kg(20.0), 20)
# use 8mm pitch x 12mm width Browning belt, 28 to 112 teeth. FD=3.15, 11.23

      
if 1:
    calc('PacSci B-404-C + Micron UltraTrue 115 5:1 + Sx30',
         ((5000, min(35.3, 60*0.66), min(13.9, 30*0.66)),),
         5, 12.5+6, 16)
    # 36 kw at 300 v: 123 amps
    # Using NiMH at 7.2v x 40a: 42 series x 3 parallel, 126 packs total (* $19 is $2400)

  
if 1:
    calc('PacSci D083A',
         ((500, 160, 50.4),),
         1, 28.8, 18)
    # 36 kw at 300 v: 123 amps
    # Using NiMH at 7.2v x 40a: 42 series x 3 parallel, 126 packs total (* $19 is $2400)


if 0:
    calc('Unicycle: MagMotor S400',
         ((0, conv_inlb_nm(3720/16), 100*conv_inlb_nm(3720/16/570)),
          (4900, conv_inlb_nm(3720/16), 100*conv_inlb_nm(3720/16/570))),
         16, conv_lbm_kg(6.7), wheel_diam=24, nmotors=1)


if 0:
    calc('Unicycle: PMG 080',
         ((0, conv_inlb_nm(2320/16), 115*conv_inlb_nm(4.67/16)),
          (6000, conv_inlb_nm(2320/16), 115*conv_inlb_nm(4.67/16))),
          16, conv_lbm_kg(7.5), wheel_diam=24, nmotors=1)

if 0:
    calc('Unicycle: NPC-2212',
         ((0, conv_inlb_nm(180), conv_inlb_nm(60.1)),
          (210, conv_inlb_nm(60), conv_inlb_nm(60.1))),
          1, conv_lbm_kg(5.1), wheel_diam=24, nmotors=2)

if 0:
    calc('Unicycle: Magmotor 28-150 + TWM3M',
         ((0, conv_inlb_nm(878), conv_inlb_nm(100)),
          (420, conv_inlb_nm(878/2), conv_inlb_nm(100))),
          3, conv_lbm_kg(4.5), wheel_diam=20, nmotors=1)

if 1:
    calc('Unicycle: MagMotor S400 2:1 right-angle gear reducer and timing belt',
         ((0, conv_inlb_nm(3720/16), 100*conv_inlb_nm(3720/16/570)),
          (4900, conv_inlb_nm(3720/16), 100*conv_inlb_nm(3720/16/570))),
         2 * 72/20, conv_lbm_kg(6.7), wheel_diam=8, nmotors=1)

if 1:
    calc('Unicycle: MagMotor S400 2:1 right-angle gear reducer and chain',
         ((0, conv_inlb_nm(3720/16), 100*conv_inlb_nm(3720/16/570)),
          (4900, conv_inlb_nm(3720/16), 100*conv_inlb_nm(3720/16/570))),
         2 * 72/12, conv_lbm_kg(6.7), wheel_diam=14, nmotors=1)

if 1:
    calc('Unicycle: MagMotor S400',
         ((0, conv_inlb_nm(3720/16), 100*conv_inlb_nm(3720/16/570)),
          (4900, conv_inlb_nm(3720/16), 100*conv_inlb_nm(3720/16/570))),
         14/1.625, conv_lbm_kg(6.7), wheel_diam=14, nmotors=1)


if 0:
    calc('EBike http://www.electricbikessys.com/Scooter_Motor.htm',
         ((915, 50.1, 31.1),),
         1, motor_weight=10, wheel_diam=11, nmotors=1)
          


calc('BS Etek',
     ((0, conv_inlb_nm(330*1.14), conv_inlb_nm(230*1.14)),
      (3000, conv_inlb_nm(330*1.14), conv_inlb_nm(160*1.14))),
      3, conv_lbm_kg(20.0), 8, nmotors=1)

calc('Clifton C23-L40',
     ((6000, 1.765, 0.191),),
     14, conv_lbm_kg(3.0), 8, nmotors=1)
      
calc('Astroflight Cobalt 60',
     ((0, 100*conv_inlb_nm(80/16), 35*conv_inlb_nm(80/16)),
      (17*40, 100*conv_inlb_nm(80/16), 35*conv_inlb_nm(80/16))),
     gear_ratio=2.5, motor_weight=conv_lbm_kg(36/16), wheel_diam=18, nmotors=1)

calc('Daedal Parker GV + BE233F + PER 60 mm',
     ((0, min(4.48*20, 88), min(1.49*20,44)),
      (6000/20, min(4.48*20, 88), min(1.49*20,44))),
     gear_ratio=1, wheel_diam=12, nmotors=1)
# thick: 134 mm


if 1:
    calc('sdp-si d50r10-067 + s9134a-pg010 ',
         ((0, conv_inoz_nm(825), conv_inoz_nm(165)),
          (5000, conv_inoz_nm(825), conv_inoz_nm(165))),
         gear_ratio=10, motor_weight=conv_lbm_kg(8.6+7), wheel_diam=10, nmotors=1)
    # torque constant = 21.6 oz-in / A
    # peak current = 825/21.6 = 38 A
    # voltage constant = 16 V / krpm
    # max voltage = 5*16 = 80
    
calc('PMB32C + Thompson NTR34-015',
     ((0, 6.8, 3.5),
      (5000, 6.8, 3.5)),
     gear_ratio=15, wheel_diam=12, nmotors=1, motor_weight=3.4+4.4)


if 1:
    calc('Unicycle: EV Warrior',
         ((0, conv_inlb_nm(1400/16), conv_inlb_nm(400/16)),
          (4900, conv_inlb_nm(1400/16), conv_inlb_nm(400/16))),
         72/20 * 72/16, conv_lbm_kg(4), wheel_diam=11.5, nmotors=1)



if 1:
    calc('Unicycle: MagMotor S28-150',
         ((0, 200*conv_si_torque(5.3,'ozf in'), 37*conv_si_torque(5.3, 'ozf in')),
          (5500, 100*conv_si_torque(5.3, 'ozf in'), 37*conv_si_torque(5.3, 'ozf in'))),
         72/10, conv_lbm_kg(3.8), wheel_diam=11.5, nmotors=1)
    calc('Unicycle: MagMotor S28-150',
         ((0, 200*conv_si_torque(5.3,'ozf in'), 37*conv_si_torque(5.3, 'ozf in')),
          (5500, 100*conv_si_torque(5.3, 'ozf in'), 37*conv_si_torque(5.3, 'ozf in'))),
         7.14*(72/28), conv_lbm_kg(3.8), wheel_diam=16.0, nmotors=1)
    calc('Unicycle: MagMotor S28-150',
         ((0, 200*conv_si_torque(5.3,'ozf in'), 37*conv_si_torque(5.3, 'ozf in')),
          (5500, 100*conv_si_torque(5.3, 'ozf in'), 37*conv_si_torque(5.3, 'ozf in'))),
         10, conv_lbm_kg(3.8), wheel_diam=16.0, nmotors=1)


if 1:
    calc('Unicycle: NPC B81',
         ((0, conv_si_torque(550,'lbf in'), conv_si_torque(550, 'lbf in')),
          (180, conv_si_torque(550,'lbf in'), conv_si_torque(550, 'lbf in'))),
         1, conv_lbm_kg(17), wheel_diam=24, nmotors=1)


if 1:
    tc=conv_si_torque(27,'ozf in')
    calc('Scooter: PMG132',
         ((0, 500*tc, 200*tc),
          (3000, 500*tc, 200*tc)),
         72/32, conv_lbm_kg(24.8), wheel_diam=24, nmotors=2)

    calc('Unicycle: PMG132',
         ((0, 500*tc, 200*tc),
          (3000, 300*tc, 200*tc)),
         72/28, conv_lbm_kg(24.8), wheel_diam=29, nmotors=1)


if 1:
    tc=conv_si_torque(4.7,'ozf in')
    calc('Scooter: PMG080',
         ((0, 250*tc, 78*tc),
          (6000, 250*tc, 78*tc)),
         8.0, conv_lbm_kg(7.5), wheel_diam=20, nmotors=2)


if 1:
    tc=conv_si_torque(4.7,'ozf in')
    calc('Unicycle: PMG080',
         ((0, 250*tc, 78*tc),
          (6000, 250*tc, 78*tc)),
         20.0, conv_lbm_kg(7.5), wheel_diam=20, nmotors=1)

if 1:
    calc('Unicycle: e-TORQ 7 inch',
         ((0, conv_inlb_nm(200), conv_inlb_nm(20)),
          (2500, conv_inlb_nm(150), conv_inlb_nm(20))),
         10.0, 0.0, wheel_diam=20, nmotors=1)
    
    calc('Unicycle: e-TORQ 14 inch',
         ((0, 1.073 * 50, 1.073 * 13.1),
          (1100, 1.073*50, 1.073 * 13.1)),
         1.0, 0.0, wheel_diam=20, nmotors=1)

if 1:
    calc('Ball Balance: MagMotor S28-150',
         ((0, 200*conv_si_torque(5.3,'ozf in'), 37*conv_si_torque(5.3, 'ozf in')),
          (5500, 100*conv_si_torque(5.3, 'ozf in'), 37*conv_si_torque(5.3, 'ozf in'))),
         1, conv_lbm_kg(3.8), wheel_diam=3.0, nmotors=2)

if 1:
    calc('Big Segway: SC 10-1672',
         ((0, kv_to_kt(1200/230)*93.3, kv_to_kt(1200/230)*11,),
         (1200, kv_to_kt(1200/230)*93.3, kv_to_kt(1200/230)*11,)),
         1, conv_lbm_kg(95), wheel_diam=26, nmotors=2)
