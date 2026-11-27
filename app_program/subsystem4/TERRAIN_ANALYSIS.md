# Buddy 4 terrain analysis

Buddy 4 estimates the height of a hump under the right wheel, counts completed crossings and reports the highest completed peak since START. In the host test, two different 10 mm hump profiles each produced one crossing and a 10 mm live peak. The deployment record does not pair a car reading with an independently measured obstacle height, so the physical height error is unknown. [Crossing tests](../../testing/subsystem4/imu_host_main.c) · [Hardware evidence](../../docs/compliance/evidence/baseline-deployment-20261119.json)

## How the car estimates height

The course configuration assumes the left wheel stays on level ground. The GY-511 accelerometer supplies roll and pitch. Wheel-encoder feedback indicates whether the car is moving forwards or pivoting. At startup, 100 accepted, stationary samples establish the level reference. The estimator takes the median of three accepted roll samples and uses filtered pitch. With the configured wheel spacing of 116 mm, right-wheel rise in millimetres is `116 × sin(roll) × cos(pitch)`. At zero pitch, a 10 mm rise corresponds to about 4.95° of roll. [Runtime configuration](buddy4.c) · [Calibration](imu_calibration.c) · [Terrain processing](terr_data.c)

Five level samples within ±3 mm arm the detector. A rise of at least 5 mm with forward travel starts a crossing. To count once, the car must travel at least 5 mm while at or above that height, then return to the level band for five samples. A pivot, reverse travel or invalid odometry discards an unfinished crossing. Only a completed crossing can raise the stored run maximum. The dashboard can show live height before completion. [Hump detector](hump_detection.c) · [Telemetry fields](DATA_FORMAT.md)

The motion label answers a different question from the hump count. A car travelling faster than 0.01 m/s on a pitch above +4° can report *climbing*, and below −4° it can report *descending*. A one-wheel rise can leave pitch near zero, so *moving* and an active hump can appear together. Ordinary motion labels require three consecutive matching classifications. An impact candidate has immediate priority. Turn rate comes from changes in calibrated magnetic heading. [Motion classifier](events_motion.c) · [Classification and validity checks](terr_data.c)

## Host test results

The crossing tests feed synthetic acceleration and encoder travel into the production C code. For both short profiles, samples arrive every 20 ms and each step advances 2 mm, equivalent to 100 mm/s. The profiles below give right-wheel elevation in millimetres before the final level samples. [Test inputs and assertions](../../testing/subsystem4/imu_host_main.c)

| Input | Observed software result |
| --- | --- |
| Gradual rise and return, `4, 8, 10, 10, 8, 4, 0` | One completed crossing. The maximum live height was 10 mm. |
| Shorter crest, `0, 7, 10, 10, 0, 0, 0` | One more completed crossing. The maximum live height was again 10 mm. |
| Isolated 20 ms roll excursion, then level travel | The detector stayed inactive. |
| Commanded pivot between crossings, with apparent roll and encoder travel | The count stayed at one through the pivot. Five level samples rearmed the detector for the second crossing. |
| Completed 10 mm and 15 mm rises, then a 20 mm candidate interrupted by reverse travel | The stored maximum stayed at 15 mm. |
| Both wheels cross a simulated longitudinal hump, with pitch changing and roll zero | The longitudinal profile counted one. The course's right-wheel profile counted zero. |

To repeat the processing tests, run `./tools/test-buddy4.ps1` in PowerShell from the repository root. The controller and transport tests also check that the cumulative count and peak survive a telemetry enqueue failure and that stale IMU data reaches the controller's fault path. [Test guide](../../testing/subsystem4/README.md) · [Recorded test output](../../testing/change_requirement_1/logs/2026-11-19_tests_buddy4.txt)

## What the evidence establishes

The tests generate roll from the prescribed height using the same wheel geometry as the estimator. Recovering 10 mm verifies the calculation, filtering and crossing rules under those inputs. The course profile rejects roll at or above 10° in magnitude. With 116 mm wheel spacing and zero pitch, 10° corresponds to about 20.1 mm of rise. The estimator can therefore miss a taller obstacle even when the car physically crosses it. [Geometry and rejection thresholds](hump_detection.c)

Wheel spacing, tyre compression, mounting angle and acceleration can all affect the estimate on the car. The 19 November deployment record confirms boot and telemetry while stopped. It contains no reference height or labelled series of crossings from which to calculate height error or a physical detection rate. [Deployment record](../../docs/compliance/evidence/baseline-deployment-20261119.json) · [Archived physical comparison procedure](../../docs/PHYSICAL_VALIDATION.md)
