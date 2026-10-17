# A3 Navigation in VR — report draft

**Course:** MAC0623 / MAC6923, 3D Interaction in Mixed Realities  
**Name:** [your name]  
**Techniques:** 1 = World-in-Miniature (WIM), 2 = left-joystick locomotion

> Delete this box before you export the PDF. It is a guide, not part of the report.  
> Target length is 2–4 pages. Fill every `[bracket]`, then delete the brackets and the sentences that start with *Guide:*.  
> Do not invent means. Compute them from your CSV. The prediction in Discussion must be answered with “supported” or “not supported.”  
> If you are in MAC0623, delete the last section. If you are in MAC6923, keep it and replace the citation note with a sentence you have actually read.

---

## 1. Task and environment

The task is waypoint navigation. A green circle appears on the floor of a house. The participant moves their viewpoint to that circle and confirms. A new circle then appears, at least 6 m from the head, and the trial repeats. This is the same trial structure as A1/A2, applied to the viewpoint instead of a docked cube.

The environment is a 16 × 16 m house on a 20 × 20 m lot, large enough that the target cannot be reached by leaning. A2’s docking volume was about 2 m across; the minimum beacon distance here is 6 m. The house has four 8 × 8 m rooms and no roof, so the rooms stay readable in the miniature:

| Room | Floor | Landmark |
|---|---|---|
| Northwest, living room | Beige | Red sofa and floor lamp |
| Northeast, bedroom | Blue | Blue bed and wardrobe |
| Southwest, kitchen | Yellow | Yellow counters and fridge |
| Southeast, study | Wood | Desk and bookshelf |

Walls are 2.55 m tall, with doorways between rooms and one entrance on the south wall. The participant starts in the living room. Walls do not collide with the user in either technique, so a detour through a wall is possible and will show up in `path_ratio`.

**Confirm radius: 1.0 m, horizontal.** The beacon is a green disc of that radius. The on-screen `dPos` readout is the horizontal distance from the head to the center of the disc, and it turns green inside the radius. One meter is the middle of the suggested 0.75–1.5 m range. On an 8 m room it is large enough to confirm without standing on an exact point, and small enough that the participant still has to reach the room the disc is in. Confirmation is the right grip (squeeze), Enter, or the Confirm button. The right trigger is reserved for grabbing the WIM marker, so it is not the confirm button.

*Guide: add one screenshot of the house and one of the miniature in the hand.*

## 2. Technique 1 — World-in-Miniature

WIM is the baseline from Class 9, navigation only. The floor, walls, furniture, and beacon live in a `world` group. The camera and controllers live in a separate rig. WebXR overwrites the camera pose, so the viewpoint moves by translating the rig.

The miniature is `world.clone()`, scaled by `0.35 / 20` and parented to the left controller grip, a few centimeters above the hand. The 20 m lot is about 35 cm across in the hand; the house is about 28 cm. Cloned lights are removed. The beacon is copied into the clone at the start of each trial.

The user is a red person marker in the miniature, about 5 cm tall in the hand, rotated to the head’s yaw. While it is not grabbed, its XZ position is the head’s position in world coordinates. The right trigger ray grabs it. **The viewpoint moves when the trigger is released, not while the marker is dragged.** On release, the rig is shifted so the head, not the rig origin, lands on the marker’s XZ position.

**Miniature orientation.** The miniature is fixed to the left grip with no extra yaw. It is not world-north-up, and it is not track-up with the head. Turning the wrist turns the map. That choice keeps the mapping “miniature-local equals world-local,” but the user has to align the hand before reading which way a room is.

## 3. Technique 2 — design draft (before implementation)

*Guide: this section is the required pre-build draft. If you have a photo or sketch from before you coded, insert it here and keep the text. If you only have this writeup, say so honestly.*

**State machine**

1. Idle. The left thumbstick is inside a dead zone of 0.15, or the technique is not selected. The rig does not move.
2. Move. Stick deflection beyond the dead zone sets a direction in the horizontal plane of the head’s yaw. Stick up is forward, stick right is strafe right. Speed is 2.6 m/s times stick magnitude. There is no collision response and no teleport.
3. Confirm. If horizontal distance to the beacon is ≤ 1.0 m and the right grip is pressed, the trial ends. Otherwise the press does nothing.

Direction comes from the head, not from the controller, so looking and steering stay the same act. Speed is constant rather than acceleration-based, so travel time is mostly distance divided by 2.6 m/s plus the time spent searching.

**Justification.** WIM makes every move a switch between the egocentric view and a second, hand-scaled frame of reference. Class 9 treats that second frame as a cost: the user must look down, interpret the map, grab a proxy, and only then move. Joystick locomotion never leaves the egocentric frame. Direction is where the head is facing and speed is the stick, which is the same mapping as walking. The predicted tradeoff is wayfinding efficiency against overview. The beacon is a circle on the floor, hidden by 2.55 m walls, so a joystick user has to search rooms and should produce a higher `path_ratio`. WIM shows the circle on the map, so the route should be closer to the straight line, at the cost of the frame switch and a discrete jump. Joystick also keeps spatial continuity — the user experiences the path — and pays for it with continuous vection. WIM avoids that vection by jumping only on release.

**What was built.** The implementation matches the draft: left stick, head yaw, 2.6 m/s, dead zone 0.15, no wall collision, same confirm rule as WIM. A desktop WASD fallback exists for development. Headset trials used the stick.

## 4. Instrumentation

One row is stored in memory per confirmed trial and downloaded as CSV at the end of each participant. A confirm outside the circle does not write a row. Changing the participant id resets the trial counters; it does not delete rows already stored, so the CSV is downloaded before the id is changed.

| Field | Definition in this build |
|---|---|
| `participant_id` | Anonymous id typed before the trials (P01, P02, …). |
| `technique` | 1 = WIM, 2 = joystick, taken from the selector at confirm time. |
| `trial_number` | 1, 2, … counted separately inside each technique. |
| `presentation_order` | 1, 2, 3, … in the order trials were confirmed for that participant. |
| `completion_time_s` | Seconds from beacon spawn to a confirm inside the circle. |
| `path_length` | Sum of horizontal head displacement per frame, in meters. |
| `straight_line_distance` | Horizontal distance from the head to the beacon at spawn. |
| `path_ratio` | `path_length / straight_line_distance`. 1.0 is a direct route. |

**Path rule, same for both techniques.** Every frame adds the horizontal distance the head moved, including physical walking inside the tracking space. A WIM release is one jump, and that jump’s length is added in the frame the rig moves. A joystick path is the sum of the small steps. Because both use the same rule, a WIM jump straight onto the circle gives `path_ratio` near 1, and a joystick search gives a ratio above 1. Vertical motion is ignored.

## 5. Data collection

Minimum design: 2 participants × 2 techniques × 2 trials = 8 rows. The selector does not enforce order. Counterbalancing was applied by hand, ABBA, as in A1/A2:

| Participant | Order of the four trials |
|---|---|
| P01 (odd) | WIM, Joystick, Joystick, WIM |
| P02 (even) | Joystick, WIM, WIM, Joystick |

*Guide: if you actually used another order, replace this table. `presentation_order` in the CSV should match it. Two trials per technique is the assignment minimum; if you collected more, say so and use all of them.*

Each person was told [one sentence: what you said they had to do, and that they could walk through walls]. Practice before the logged trials: [yes/no, and how many]. Headset: [Quest / other].

## 6. Results

*Guide: from the CSV, for each technique, report mean and standard deviation of `completion_time_s` and of `path_ratio`. Also report mean `path_length` and mean `straight_line_distance` so the ratio is interpretable. n = 4 cells per technique if you stopped at the minimum (2 people × 2 trials).*

| Technique | n | Time (s), mean ± sd | Path length (m), mean | Straight line (m), mean | Path ratio, mean ± sd |
|---|---|---|---|---|---|
| 1 WIM | [n] | [ ] ± [ ] | [ ] | [ ] | [ ] ± [ ] |
| 2 Joystick | [n] | [ ] ± [ ] | [ ] | [ ] | [ ] ± [ ] |

[One sentence on which technique was faster.] [One sentence on which technique had the lower path ratio.] [If one person accounts for most of the difference, say so. With n this small, do not claim a general advantage.]

*Guide: a scatter of time vs path_ratio, one point per trial, is enough if you want a figure. Color by technique.*

## 7. Discussion

The prediction was that joystick locomotion would produce a **higher path ratio** than WIM, because the user stays in one egocentric frame and never sees the beacon until they enter its room, while WIM shows the room on the map before the jump. Completion time was expected to favor WIM only when the frame switch was cheaper than the search; joystick was expected to feel more continuous and more sickness-prone.

**The prediction was [supported / not supported].**

[Two or three sentences that use your actual means. Example shape, delete if it does not match the numbers: WIM’s mean path ratio was X and joystick’s was Y, so the overview did / did not reduce detours. Mean time was A s for WIM and B s for joystick, so the frame switch did / did not cost more than searching.]

Why this pattern is plausible in this build, whether or not it matched the prediction:

- The circle sits on the floor behind 2.55 m walls. Joystick users can miss it and walk through several rooms. That inflates `path_ratio` even when they move quickly.
- WIM’s jump is efficient only if the grab lands inside the circle. A miss still counts as path length, then the user grabs again. Overshoot would raise both time and ratio.
- There are no collisions. Joystick can cut through walls, which shortens paths relative to a real house and can make joystick look more efficient than it would with collision.
- Two trials per technique is not enough to separate learning from the technique. The second trial of each technique is confounded with practice. ABBA reduces, but does not remove, that effect. Both techniques were new.
- The miniature rotates with the wrist. A participant who does not align the hand can misread the map and jump to the wrong room. That is a WIM-specific error, not a joystick one.
- [Anything you actually saw: missed grabs, people walking past the circle, cybersickness comments, the red marker hard to hit.]

*Guide: the last paragraph should be your own observation from the sessions. Graders want a real account if the prediction failed, not a restatement of the hypothesis.*

---

## 8. MAC6923 only — literature and statistics

*Guide: delete this whole section for MAC0623. For MAC6923, do not submit the bracketed instructions.*

**Literature.** Stoakley, Conway, and Pausch (1995) describe navigation by moving a representation of the user inside a miniature of the world, while the user still sees the full-scale scene. The claim to engage is their two-scale interface: an egocentric view plus an exocentric miniature. That second scale is the cost this study isolates. Joystick locomotion refuses the exocentric scale, so it should save the attention switch and lose the overview that makes WIM’s routes short. [Add one sentence in your own words that says whether your path_ratio result agrees with that cost. Cite the CHI ’95 paper properly. Read the page you cite; do not keep this paraphrase if you have not opened the paper.]

Stoakley, R., Conway, M. J., & Pausch, R. (1995). Virtual reality on a WIM: Interactive worlds in miniature. *Proceedings of CHI ’95*, 265–272.

**Statistics.** With two participants and two trials per cell, a paired comparison is descriptive, not a population inference. [If you report a test: name it (for example a paired Wilcoxon or a paired t on the two participant means), give the statistic and p, and say that it is underpowered.] Confounds that a test does not remove:

1. **Novelty.** Neither technique was practiced to asymptote.
2. **Learning across the four trials.** Trial 2 of a technique is easier than trial 1 even if the technique is worse. ABBA balances which technique is first, not the improvement inside a technique.
3. **Unequal practice.** If one participant needed extra unlogged tries on WIM to hit the marker, that practice is invisible in the CSV.
4. **No collisions.** Joystick distances are shorter than they would be in a solid house.
5. **Beacon visibility.** The green disc is obvious in the miniature and hidden in the full-scale house, so the task favors an overview technique by construction.

---

## Checklist before you export

- [ ] Every `[bracket]` is gone.
- [ ] The guide paragraphs are gone.
- [ ] `path_ratio` is in the results table, not only time.
- [ ] Discussion says “supported” or “not supported” in a sentence of its own.
- [ ] The order you actually ran matches section 5.
- [ ] The CSV in the zip is the one these numbers came from.
- [ ] Technique 2 section still contains the state machine (the design draft).
- [ ] MAC6923 section is deleted or finished, not left as instructions.
