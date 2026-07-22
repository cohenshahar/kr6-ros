# kr6-ros — מערכת ה-VLA על KR6, ב-ROS2

המרת מערכת הניסוי (MuJoCo scene_v2 + ACT/SmolVLA + executor) לארכיטקטורת ROS2 Humble
של שלושה נודים: סימולציה / מדיניות / ביצוע. פייתון בלבד (rclpy).
תוכנית מלאה: [PLAN_ROS.md](PLAN_ROS.md) · הערות שחר: [FEEDBACK.md](FEEDBACK.md)

## סטטוס שערים
| שער | תיאור | סטטוס |
|---|---|---|
| R0 | שלד: 5 חבילות נבנות, 3 נודים עולים | ✅ 22/07 |
| R1 | sim_node טוען scene_v2, reset/apply, וידאו | ✅ 22/07 |
| R2 | אפיזודת ACT מקצה-לקצה ב-lockstep | ✅ 22/07 — זהות 13/13 |
| R3 | שחזור 23/24 של ACT C0 מול ה-JSON הקיים | ✅ 22/07 — 23/24, הצלחות 24/24 זהות |
| R4 | SmolVLA בפרמטר | ⏸ ממצא: המודל סטוכסטי מטבעו — ממתין להכרעת שחר על קריטריון (FEEDBACK.md) |
| R5 | מצב זרימה חופשית + מדידות | ⬜ |
| R6 | איסוף oracle דרך ROS + roundtrip | ⬜ |
| R7 | אימון מדאטה שנאסף ב-ROS + eval | ⬜ |

## בנייה והרצה
```bash
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch kr6_eval kr6_system.launch.py
```
