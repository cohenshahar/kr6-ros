# תוכנית ההמרה ל-ROS2 — מערכת KR6 ‏(kr6-ros)

**המשימה (שחר, 22/07/2026):** הביצוע במלואו בפייתון על סביבת עבודה של ROS. דדליין: ראשון בבוקר (27/07). עדכון גיטהאב פעמיים ביום + בדיקת הערות של שחר לפני כל דחיפה.

**הכרעות שחר (22/07, בשיחה):**
1. היקף: מערכת ה-KR6 בלבד (scene_v2 + מדיניות + executor + eval). ‏LIBERO נשאר כפי שהוא; אימון נשאר lerobot offline.
2. ארכיטקטורה: 3 נודים מלאים — סימולציה / מדיניות / executor.
3. סנכרון: שני מצבים — lockstep דטרמיניסטי (שער אימות: שחזור מדויק) + זרימה חופשית (תזמון ריאליסטי).
4. מיקום: ריפו חדש ייעודי `kr6-ros`. הערות שחר: עריכת `FEEDBACK.md` (עריכה = הוראה).

## סביבה (נמדד, לא מונחש)
- Ubuntu 22.04.5, ‏ROS2 **Humble** ב-`/opt/ros/humble` ✓
- מקור להמרה: ‏`vla-sim-playground/nt16/` — ‏core.py ‏(394 ש'), executor.py ‏(67 ש'), oracle.py ‏(410 ש'), visibility.py ‏(120 ש'); ‏`act/eval_act.py` ‏(243 ש'), ‏`act/eval_smolvla.py`, ‏`act/nt17_gen_eval.py` ‏(פאצ'ים של תנאי הכללה).
- צ'קפוינטים: ‏`act/runs/act_boxv2/…/last/pretrained_model` ‏(ACT 23/24), ‏`act/runs/smolvla_boxv2_3cam/…` ‏(SmolVLA 19/24).
- אמת-מידה לשחזור: ‏`act/results/nt17_gen/nt17_eval_act_C0_base.json` — ‏23/24, פר-אפיזודה.

## ארכיטקטורה

```
                    ┌─────────────┐   /kr6/obs (Image ×N + JointState)   ┌──────────────┐
                    │  sim_node    │ ────────────────────────────────────▶ │ policy_node  │
                    │ (MuJoCo      │                                       │ (ACT/SmolVLA │
                    │  scene_v2)   │ ◀──┐                                  │  lerobot)    │
                    └─────────────┘    │ /kr6/joint_cmd (+vacuum)         └──────┬───────┘
                          ▲            │                                          │ /kr6/action_chunk (7D×50)
                          │       ┌────┴─────────┐                                │
                     srv: reset   │ executor_node │ ◀──────────────────────────────┘
                     srv: apply   │ (flange-IK    │
                                  │  axis-only)   │
                                  └───────────────┘
```

- **kr6_msgs** — ‏ActionChunk.msg, ‏Observation.msg, ‏EpisodeResult.msg; ‏srv: ‏Reset ‏(seed, task, condition), ‏ApplyCmd ‏(lockstep), ‏GetChunk ‏(lockstep).
- **מצב lockstep:** לולאת ה-eval קוראת לשירותים בזה-אחר-זה — סדר פעולות זהה 1:1 ללולאה הקיימת ⇒ אותם seeds, אותן תוצאות.
- **מצב זרימה:** אותם נודים, timers + topics ‏(QoS sensor-data), בלי המתנה הדדית. מדידה: קצב בקרה, גיל-תצפית (השהיה), שיעור הצלחה.
- **וידאו:** כל eval שומר מצלמת מדיניות **וגם** מבט spectator רחב ‏(`_spec`) — לפי הכלל החדש בתוכנית הראשית.

## חבילות ב-`ros2_ws/src/`
| חבילה | תוכן |
|---|---|
| `kr6_msgs` | ממשקים בלבד (msg/srv) |
| `kr6_sim` | ‏sim_node — עוטף את SceneV2 + oracle setup + visibility + הקלטת וידאו |
| `kr6_policy` | ‏policy_node — טעינת צ'קפוינט lerobot, פרמטר `policy:=act\|smolvla` |
| `kr6_executor` | ‏executor_node — ‏preroll + execute_window (flange-site IK axis-only + lead joint) |
| `kr6_eval` | ‏eval_node (מתזמר אפיזודות, כותב JSON באותה סכמה) + קובצי launch |

## שערים (כל שער = קומיט + דחיפה)

**GATE R0 — שלד עומד.** ‏5 החבילות נבנות ב-`colcon build` נקי; ‏launch מרים 3 נודים ריקים; ‏`ros2 topic list` מציג את ה-topics. ✔ פלט build + topic list נשמר ל-`results/r0_build.txt`.

**GATE R1 — סימולציה כנוד.** ‏sim_node טוען scene_v2, ‏srv reset(seed) משחזר פריסה זהה לזו של `SceneV2(system)` הישיר (השוואת qpos/מיקומי אובייקטים בטולרנס 1e-9); ‏ApplyCmd מריץ חלון פיזיקה. ✔ ‏`results/r1_reset_parity.json` + וידאו spectator של אפיזודת oracle דרך ROS.

**GATE R2 — שרשרת מלאה.** אפיזודת ACT אחת מקצה-לקצה ב-lockstep (seed 50000) מצליחה, והמסלול זהה לריצת הפייתון הישיר על אותו seed. ✔ ‏`results/r2_first_episode.json` + וידאו.

**GATE R3 — שחזור מלא (שער האמת).** ‏24 אפיזודות ACT C0 ב-lockstep ⇒ ‏**23/24**, עם התאמה פר-אפיזודה מול `nt17_eval_act_C0_base.json`. סטייה נומרית אם תתגלה — נעצרים, מתעדים, לא "מעגלים". ✔ ‏`results/r3_act_c0_ros.json` + טבלת דיפ.

**GATE R4 — החלפת מדיניות בפרמטר.** אותו launch עם `policy:=smolvla` ⇒ ‏**19/24**. ✔ ‏`results/r4_smolvla_c0_ros.json`.

**GATE R5 — זרימה חופשית.** אותה מערכת ב-free-run: מדידת קצב בקרה בפועל, גיל תצפית, ושיעור הצלחה על 24 האפיזודות (אין ציפייה מוגדרת — מודדים ומתעדים). ✔ ‏`results/r5_freerun.json` + ניתוח קצר.

**GATE R6 — איסוף oracle דרך ROS (אישור שחר 22/07: "גם הרצות ה-oracle").** ‏oracle_node (או mode ב-eval_node) מריץ את אוסף ההדגמות של nt16 דרך שירותי הסימולציה — אותם seeds של איסוף (20000+), אותה סכמת npz. אימות: ‏roundtrip על ההדגמות שנאספו-דרך-ROS ⇒ שיעור הצלחה ≥ ‏25/30 של ה-RT הקיים. ✔ ‏`results/r6_collect_roundtrip.json`.

**GATE R7 — אימון מהצינור החדש (אישור שחר 22/07: "והאימונים").** המרת ההדגמות שנאספו ב-R6 לפורמט lerobot ואימון ACT קצר (ריצת לילה על ה-3060; האימון עצמו offline — כפי שהוכרע, רק הדאטה מגיע מ-ROS) ← ‏eval ב-lockstep. אימות: המדיניות המאומנת-מחדש משיגה שיעור בסדר הגודל של הבסיס (אין ציפייה מספרית קשיחה — מתעדים). ✔ ‏`results/r7_retrain_eval.json`.
**גבול חשוב:** זה אימון לאימות-צינור בלבד. החלטות nt18 (האם/איך לאמן מחדש עם רנדומיזציה) נשארות מוקפאות עד הישיבה המשותפת — הכלל שנקבע מראש בעינו.

**רזרבה (אם נשאר זמן לפני ראשון):** תנאי nt17 (צבע/גודל) כפרמטרי launch; ‏rviz config להצגת המערכת.

## פרוטוקול עבודה
- דחיפה לגיטהאב פעמיים ביום (בוקר ~09:00, אחה"צ ~17:00). **לפני כל דחיפה:** ‏`git fetch` + קריאת FEEDBACK.md והקומיטים של שחר — עריכה של שחר = הוראה מחייבת.
- נגמרו הטוקנים ⇒ המתנה 4 שעות והמשך מאותו צעד (הסטטוס נשמר ב-TODO_STATE.md).
- כל שער שנסגר — שורת סטטוס ב-README (טבלת התקדמות) + קומיט.
- אין הערכות זמן פרט לדדליין שנקבע; מתקדמים שער-אחר-שער.
