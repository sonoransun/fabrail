# fabrail references

The numbered source list for every fabrail document. Documents cite an entry as a bracketed tag such as [R1]; the traceability guard (FAB-SW-007, [SPECIFICATION §6](SPECIFICATION.md)) fails if a tag does not resolve here.

**How to read an entry.** Citation, then *Why it matters* for fabrail, then the requirements (FAB IDs) and physics sections it supports, then a locator.

- **Numbers are permanent.** A number is never reused or renumbered. A new source takes the next free number and goes under the heading it belongs to, so numbers may run out of order inside a group.
- **Standards.** The edition is the one whose numbers fabrail uses; the entry says when the research read a draft or a secondary summary instead of the published text. "Fixed in SAFETY.md §10" marks an edition pinned there; "edition to confirm" means none is pinned yet. No entry implies compliance or certification. The only wording used anywhere is "checked in simulation against numeric limits from [edition]" (SPECIFICATION §4).
- **Locators** are a DOI, an arXiv identifier or a host/path written without a scheme, so the offline site never makes an external request; add the https scheme to open one.
- **Markers.** *Secondary*: a summary, vendor or news page rather than the primary document. *Not fetched*: cited from memory during the research; bibliographic details are to be confirmed. *[UNCERTAIN]*: the research flagged the quoted number itself.

**Groups** (number ranges as first published): standards, codes and guidelines R1–R25 · magnetic levitation, magnetics and actuators R26–R49 · haptics and physical human–machine interaction R50–R58 · seismology, early warning, ground motion and structural control R59–R93 · prior art R94–R118 · embedded, firmware and software tooling R119–R136 · sovereign tier: reference installations, dampers and storage R137–R170 · sovereign tier: superconducting levitation, warning networks, blast and tsunami R171–R194 · frontier tier (🔭) R195–R215.

---

## Standards, codes and guidelines

**[R1]** CEN, *prEN 16005 Power operated pedestrian doorsets — Safety in use — Requirements and test methods*, draft text (2011). *Why it matters:* the force–time profile (400 N for ≤ 0.75 s, then 150 N, then < 80 N within 5 s), the 67 N / 1.69 J low-energy limits and Table F.2 behind the directional clamp and the monitor's force timer (FAB-PRP-004, FAB-SAF-004), and the PL c/d design targets (FAB-SAF-019). fabrail read the draft, not the published standard, so every use says "2011 draft". `otomatikkapidunyasi.com/teknik-dokumanlar/7-normlar-standartlar/prEN16005(E)-Standardi.pdf`

**[R2]** EDSF (European Door and Shutter Federation), *Guideline 06 on EN 16005*. *Why it matters:* an independent confirmation of the 67 N / 1.69 J low-energy figures, the PL c/d targets, safety distances and the per-cycle sensor test, used to cross-check [R1]. `www.edsf.com/fileadmin/inhalte/edsf/download/edsf_guideline_06_EN_16005.pdf`

**[R3]** Doorparts, *BS EN 16005 explained* (2025). Secondary. *Why it matters:* secondary EN 16005 figures (150 N; sliding doors open within 5 s on power failure). It is inconsistent on the residual force, so it is never used alone. `www.doorparts.co.uk/blogs/news/bs-en-16005-explained-keep-your-automatic-doors-compliant-in-2025`

**[R4]** ANSI/BHMA A156.10-2005, *Power Operated Pedestrian Doors*; summary (secondary) by Auto Door and Hardware. *Why it matters:* sliding doors above 160 lb are limited to V = 161/W (ft/s, lb), a constant-momentum cap of m·v ≤ 22.3 kg·m/s, which sets WALL's unsensed 0.0446 m/s (F3; FAB-SAF-002). It also gives 0.3 m/s closing, the 30 / 50 lbf forces and hold-open times. HEAVY hold-to-run lies outside its scope, which is why HEAVY momentum is stated but not capped. `www.automateddoorsolutions.com/wp-content/uploads/2018/08/ANSI-A156-10-2005.pdf`; `www.autodoorandhardware.com/Understanding-ANSI-A156.10-Automatic-Sliding-Door-Requireme-s/123399.htm`

**[R5]** ANSI/BHMA A156.19-2019 (edition fixed in SAFETY.md §10), *Power Assist and Low Energy Power Operated Doors*, read through a secondary summary. *Why it matters:* the 1.69 J kinetic-energy basis and the 67 N stall force of low-energy operation, hence the unsensed speed cap v ≤ √(2·1.69/m) = √(3.38/m) (FAB-SAF-002, FAB-PRP-004; PHYSICS §16). `www.autodoorandhardware.com/Understanding-ANSI-A156.19-Low-Energy-Door-Operator-Require-s/123397.htm`

**[R6]** ANSI/BHMA A156.38-2019, *Low Energy Power Operated Sliding and Folding Doors*, read through BHMA *Hardware Highlights* (2019) and a Construction Specifier article on speeds and hold-open time (secondary). *Why it matters:* the sliding-door counterpart of [R5]: the 1.69 J kinetic-energy basis for sliding doors, 15 lbf (67 N) stopping force, low-energy sliding speeds (DOOR closes at 0.152 m/s) and the 300,000-cycle test (FAB-SAF-002). `buildershardware.com/Portals/0/Files/2019%20Hardware%20Highlights/156.38%20HH%202019.pdf`; `buildershardware.com/ANSI-BHMA-Standards/Hardware-Highlights/A15638-2019-Low-Energy-Power-Operated-Sliding-and-Folding-Doors`; `www.constructionspecifier.com/power-operated-doors-maintaining-safe-egress-with-updated-standards/2/`

**[R7]** ISO/TS 15066:2016, *Robots and robotic devices — Collaborative robots*. *Why it matters:* body-region stiffness and effective mass (hand 75 N/mm, 0.6 kg; chest 25 N/mm, 40 kg) and the transferred-energy relation F = √(2E·k) = v·√(m·k) (Tables A.2 and A.3, Formula A.5). These give the clamp forces 290.7 N (chest), 503.5 N (hand) and 173 N through a 10 N/mm edge seal at 1.69 J (FAB-SAF-005; PHYSICS §8, §22). `www.diag.uniroma1.it/deluca/pHRI_elective/ISO_TS_15066_2016_en.pdf`

**[R8]** U.S. Department of Justice, *2010 ADA Standards for Accessible Design*, §404.2.9, read through UpCodes (secondary). *Why it matters:* the 5 lbf (22.2 N) operating force, the yardstick for manual force after a landing and for breakaway force (FAB-SAF-010, FAB-HAP-003). `up.codes/s/door-and-gate-opening-force`

**[R9]** ICC, *International Building Code* 2021 (edition fixed in SAFETY.md §10), §1010.3.2 (power-operated doors) and §1010.3.3 (special-purpose horizontal sliding doors), read through UpCodes (secondary). *Why it matters:* egress doors stay manually operable and are never locked by the seismic action, with the manual-force limits that apply after a landing (FAB-SEI-011, FAB-SEI-019). `up.codes/s/power-operated-doors`; `up.codes/s/special-purpose-horizontal-sliding-accordion-or-folding-doors`

**[R10]** NFPA 80, *Standard for Fire Doors and Other Opening Protectives*, 2022 (edition fixed in SAFETY.md §10), read with IBC §716. *Why it matters:* fire-rated doors are self- or automatic-closing, so the seismic action never opens a fire-rated DOOR (FAB-SEI-011).

**[R11]** ISO 13849-1:2023 (edition fixed in SAFETY.md §10), *Safety of machinery — Safety-related parts of control systems — Part 1: General principles for design*. *Why it matters:* performance levels (PL c/d in the core, PL e in the sovereign zones) as design targets and the Category vocabulary. The single-MCU monitor is "Cat 3 intent" (software diversity only), designed toward and never certified (FAB-SAF-012, FAB-SAF-019, FAB-EXO-501).

**[R12]** IEC 61800-5-2:2016 (edition fixed in SAFETY.md §10), *Adjustable speed electrical power drive systems — Part 5-2: Safety requirements — Functional*. *Why it matters:* the definitions of STO, SS1 and safely limited speed and position that the monitor and supervisor borrow (FAB-SAF-008, FAB-SAF-013; PHYSICS §18).

**[R13]** IEC 60730-1:2013 with amendments 1 (2015) and 2 (2020) (edition fixed in SAFETY.md §10), *Automatic electrical controls — Part 1: General requirements*, Annex H (software class B). *Why it matters:* the CRC-plus-diverse-copies pattern for the safety parameter block, used as intent only (FAB-SAF-015).

**[R14]** IEC 60335-2-116, *Household and similar electrical appliances — Safety — Particular requirements for furniture with electrically motorized parts*: an earlier edition's scope page and Ed. 2 (2025). *Why it matters:* the natural product standard for motorised cabinetry. Its numeric drawer limits are not public, so fabrail uses none of them and says so (SPECIFICATION §4, §5). `webstore.iec.ch/en/publication/62147`; `webstore.iec.ch/en/publication/110831`

**[R15]** UL 325, *Door, Drapery, Gate, Louver, and Window Operators and Systems*, read through DoorKing and LiftMaster pages (secondary), and the U.S. CPSC safety standard for automatic residential garage door operators, Federal Register, 13 July 2018. *Why it matters:* entrapment types, the 2 s reversal and monitored entrapment devices. Considered and not used (SPECIFICATION §4). `www.doorking.com/safety/ul-325/`; `www.liftmaster.com/about-liftmaster/safety`; `www.federalregister.gov/documents/2018/07/13/2018-14909/safety-standard-for-automatic-residential-garage-door-operators/`

**[R16]** ASCE/SEI 7-16, *Minimum Design Loads and Associated Criteria for Buildings and Other Structures* (2016); the ASCE 7-22 Chapter 13 changes read through a secondary summary. *Why it matters:* the design spectrum of the synthetic records (S_DS 1.0, S_D1 0.6, T_L 8 s), floor amplification 1 + 2z/h (Eq. 13.3-1) and the Chapter 17 isolation-displacement context (FAB-SEI-018, FAB-SEI-020, FAB-EXO-505; PHYSICS §19). The 7-22 changes are noted, not adopted. `www.panacheg.com/seismic-anchors/asce-7-22-chapter-13`

**[R17]** ICNIRP, *Guidelines on limits of exposure to static magnetic fields*, Health Physics (2009). *Why it matters:* 400 mT (public) and 2 T (occupational) limits, and the 0.5 mT precaution level for implant wearers behind the ≤ 0.5 mT at 10 cm stray-field target (FAB-ENV-001; PHYSICS §23). `www.icnirp.org/cms/upload/publications/ICNIRPstatgdl.pdf`; `www.icnirp.org/en/frequencies/static-magnetic-fields-0-hz/index.html`

**[R18]** ISO 14117, *Active implantable medical devices — Electromagnetic compatibility — EMC test protocols*, read through a secondary summary. *Why it matters:* implant static-field immunity at 1 mT and 50 mT, a second reason the stray-field target is 0.5 mT and why WALL and HEAVY carry shroud flags (FAB-ENV-001, FAB-EXO-104). `www.atecorp.com/compliance-standards/iso/iso-14117`

**[R19]** EN ISO 13854:2019, *Safety of machinery — Minimum gaps to avoid crushing of parts of the human body* (replaces EN 349). *Why it matters:* the finger-gap geometry target among the hardware-only design targets (FAB-SAF-019). `standards.iteh.ai/catalog/standards/cen/0b03d75b-c959-4266-8a2b-a38c1d80d57d/en-iso-13854-2019`

**[R20]** ASTM E557 (operable partitions; edition and exact title to confirm), read through Enforce and Moderco pages (secondary). *Why it matters:* partition supports deflect ≤ 1/8 in over 12 ft (≈ L/1150), the rail-sag amplitude of the irregularity model and part of the reason WALL runs a 10 mm gap (FAB-LEV-010; PHYSICS §10). `www.enforce.com.sg/2026/09/03/operable-wall-ceiling-support-requirements-singapore/`; `www.moderco.com/moveable-wall-deflection/`

**[R21]** IEC 61508, *Functional safety of electrical/electronic/programmable electronic safety-related systems*, Parts 2, 3 and 6, Ed. 2. *Why it matters:* SIL targets, the 1oo2D and 2oo3 architectures, the dependency rule and both PFH forms (conservative 1.04e-8/h; Ed. 2 ≈ 2.0e-10/h), all "designed toward, not certified" (FAB-EXO-301 to 305, FAB-EXO-905; EXOTIC §X13, §S3).

**[R22]** IEC 61784-3, *Industrial communication networks — Profiles — Part 3: Functional safety fieldbuses*, edition to confirm. *Why it matters:* the black-channel concept (CRC, sequence counter, timeout, source ID) of the exotic safety link (FAB-EXO-306; EXOTIC §X13).

**[R23]** ISO 13855, *Safety of machinery — Positioning of safeguards with respect to the approach of the human body*, edition to confirm. *Why it matters:* the separation distance S = K·(t1 + t2) + C, reported as 2.61 / 5.01 / 10.61 m for SHIELD / BASTION / ARENA (FAB-EXO-502; EXOTIC §X14).

**[R24]** ISO 14119, *Safety of machinery — Interlocking devices associated with guards*, edition to confirm. *Why it matters:* guard locking and interlocks behind the ClearVerified zone state (FAB-EXO-501).

**[R25]** ISO 12100, *Safety of machinery — General principles for design — Risk assessment and risk reduction*, edition to confirm. *Why it matters:* the risk-reduction frame for attended jog with a force cap and a compliant edge (FAB-EXO-503).

---

## Magnetic levitation, magnetics and actuators

**[R26]** S. Earnshaw, "On the nature of the molecular forces which regulate the constitution of the luminiferous ether", *Trans. Cambridge Philos. Soc.* 7, 97–112 (1842). *Why it matters:* no static arrangement of charges or magnets is stable in every direction, so fabrail's suspension needs active control (FAB-LEV-001; PHYSICS §2, §3).

**[R27]** W. Braunbek, "Freischwebende Körper im elektrischen und magnetischen Feld", *Z. Phys.* 112 (1939). Not fetched. *Why it matters:* the extension of Earnshaw's result to paramagnetic and ferromagnetic bodies (diamagnets are the exception), the reason an iron rail over permanent magnets cannot float passively (FAB-LEV-001, FAB-FRN-009).

**[R28]** "Earnshaw (1805–1888) and passive magnetic levitation" (ResearchGate), and a *Scientific Reports* (2022) paper on the stiffness-sum constraint. *Why it matters:* the modern statement that the stiffnesses of a static magnetic system sum to ≤ 0, asserted directly by `rs:plant/magnet::stiffness_sum_nonpositive` (FAB-LEV-001). `www.researchgate.net/publication/226117541_Earnshaw_1805-1888_and_Passive_Magnetic_Levitation`; `doi:10.1038/s41598-022-07988-6`

**[R29]** T. Mizuno and Y. Takemori, transfer-function analysis and design of zero-power controllers for magnetic suspension systems, *Electrical Engineering in Japan*. *Why it matters:* the zero-power law (current integral or velocity feedback) that drives the coil current to zero at equilibrium, so permanent magnets carry the weight (FAB-LEV-004, FAB-PWR-003; PHYSICS §15). `doi:10.1002/eej.10049`

**[R30]** T. Mizuno et al., modified zero-power control, *Journal of Vibration and Control* (2012). *Why it matters:* the modified zero-power band that clamps the float gap with a small bias current, fabrail's fix F1 for the 48 kg DRAWER (1.5–6.0 mm band, +0.39 A; FAB-LEV-018). `doi:10.1177/1077546311419985`

**[R31]** Aalborg University, MCE4-1024 project report on a zero-power hybrid levitation system. *Why it matters:* a worked integral-of-current model used to cross-check the hybrid element and the zero-power gains (PHYSICS §2, §15). `projekter.aau.dk/projekter/files/213558827/MCE4_1024_Report.pdf`

**[R32]** Zero-power suspension and its negative stiffness, *Control Engineering Practice* (2006). *Why it matters:* under zero-power control added load moves the carriage toward the rail. That is the m(g) law of FAB-LEV-005 (DRAWER 1.5 mm → 45.76 kg) and the DOOR +98.1 N → 4.5967 mm anchor of FAB-LEV-004. `www.sciencedirect.com/science/article/abs/pii/S0967066106001183`

**[R33]** Zero-power permanent-magnet suspension with variable flux-path control, *Actuators* 10(6), 118 (2021). *Why it matters:* one answer to the load-range problem that the DRAWER faces at 6:1 (8–48 kg), noted as an alternative to the band clamp and the seat (FAB-LEV-017, FAB-LEV-018). `doi:10.3390/act10060118`

**[R34]** Zero-power PEMS force model with a Hall-effect gap sensor, *Scientific Reports* (2025). *Why it matters:* shows a fixed-gain zero-power suspension spanning 245–1205 g, evidence for the load range a single gain set can hold (FAB-LEV-007). `doi:10.1038/s41598-025-95317-y`

**[R35]** Zero-power control with tilt resistance for multi-DOF suspension, IEEE (document 9592690). *Why it matters:* multi-DOF zero-power control, the basis of the heave, pitch and roll modes and the modal allocation (FAB-LEV-001, FAB-LEV-009). `ieeexplore.ieee.org/document/9592690/`

**[R36]** EMS levitation control overview, *Energies* 16(7), 2995 (2023). *Why it matters:* the survey behind the choice of a discrete, observer-based gap loop (FAB-LEV-007; PHYSICS §15). `www.mdpi.com/1996-1073/16/7/2995`

**[R37]** Gain-scheduled EMS control at 10 kHz sampling, *Actuators* 15(3), 151. *Why it matters:* a precedent for the 10 kHz fast group and for gains that vary with gap (FAB-LEV-007, FAB-FW-004). `www.mdpi.com/2076-0825/15/3/151`

**[R38]** Self-sensing gap estimation from PWM current ripple, *Actuators* 9(1), 17. *Why it matters:* a roadmap option for a sensorless or diverse gap channel (the inductance channel L(g), FAB-EXO-304). `mdpi.com/2076-0825/9/1/17/htm`

**[R39]** Machine Design, "Course audit: brushless linear motors" (secondary), and ETEL, iron-core versus ironless linear motors (white paper). *Why it matters:* iron-core attraction up to about 6× peak thrust, peak/continuous ≈ 2.5, zero cogging for ironless motors. Hence ironless LSMs on DRAWER, DOOR and WALL, an iron-core LSM only on HEAVY, and an ironless trim LSM on SENTINEL (FAB-PRP-003, FAB-EXO-607; PHYSICS §7). `machinedesign.com/technologies/course-audit-brushless-linear-motors`; `img.controldesign.com/files/base/ebm/controldesign/document/2022/08/1661892257638-wp_035_etel_thompson.pdf`

**[R40]** Micro-Epsilon, eddy-current displacement sensors (product pages). *Why it matters:* ≤ 0.5 µm resolution at up to 100 kHz, the basis of the gap-sensor noise presets (0.3–5 µm in band) and of the sensor-noise force gain (FAB-HAP-011; PHYSICS §13). `www.micro-epsilon.com/distance-sensors/inductive-sensors-eddy-current/`

**[R41]** RLS, LM13 magnetic linear encoder system. *Why it matters:* ±10 µm accuracy at 20 m/s, the class of absolute scale assumed beside the 1 µm/count encoder model (FAB-PRP-006; PHYSICS §13). `www.rls.si/eng/lm13-magnetic-linear-and-rotary-encoder-system`

**[R42]** Eddy-current brake force calculation, *Periodica Polytechnica Electrical Engineering*. *Why it matters:* the linear-regime eddy brake behind the progressive end zones (c_max 100 / 300 / 1500 / 8000 N·s/m) and the copper dampers of the sovereign tier (FAB-SAF-014, FAB-EXO-118; PHYSICS §9). `pp.bme.hu/ee/article/download/4761/3866/`

**[R43]** J. H. Klote, analysis of the influence of the piston effect on elevator smoke control (NIST). *Why it matters:* the piston model behind the DRAWER cabinet pressure disturbance (5.6 N with a 40 cm² leak) and the DOOR smoke ΔP (FAB-ENV-003, FAB-HAP-006; PHYSICS §11). `www.nist.gov/publications/analysis-influence-piston-effect-elevator-smoke-control`

**[R44]** P. Dupont, V. Hayward, B. Armstrong and F. Altpeter, "Single state elastoplastic friction models", *IEEE Trans. Automatic Control* 47(5), 787–792 (2002). *Why it matters:* the elasto-plastic bristle friction used for rollers, pads, the guide fin and the contents model, so presliding is modelled rather than a sign(v) jump (PHYSICS §8; FAB-EXA-009).

**[R45]** K. H. Hunt and F. R. E. Crossley, "Coefficient of restitution interpreted as damping in vibroimpact", *J. Applied Mechanics* 42(2), 440–445 (1975). *Why it matters:* the bumper contact law, which keeps impact dissipation physical and bounded (FAB-SAF-014, FAB-LEV-019; PHYSICS §8).

**[R46]** magpylib 5.2.3 documentation (changelog and force introduction), the magpylib-force add-on, and magpylib-material-response. *Why it matters:* the optional cross-check of the dipole stray-field model (extra `field`); magpylib does not model soft iron, so it cannot validate the rail reluctance and no FEM claim is made (FAB-ENV-001; SPECIFICATION §5). `magpylib.readthedocs.io/en/latest/_pages/changelog_.html`; `magpylib.readthedocs.io/en/latest/_pages/user_guide/examples/examples_force_intro.html`; `pypi.org/project/magpylib-force/`; `github.com/magpylib/magpylib-material-response`

**[R47]** pyFEMM. *Why it matters:* it is Windows-only (win32com), one reason the magnet model is a reluctance model with leakage swept in studies rather than an FEM run (PHYSICS §26). `www.femm.info/wiki/pyFEMM`

**[R48]** Carpenter Technology, selecting soft magnetic alloys (Hiperco 50). *Why it matters:* FeCo saturates at ≈ 2.3–2.4 T against ≈ 2.0 T for silicon steel, so FeCo poles raise pressure only until the rail saturates (FAB-EXO-207, FAB-EXO-103; EXOTIC §X2). `www.carpentertechnology.com/blog/a-simplified-method-of-selecting-soft-magnetic-alloys`

**[R49]** Grain-boundary-diffused NdFeB with Tb–Cu sources, *AIP Advances* 9, 125238 (2019); a related *J. Magn. Magn. Mater.* (2025) paper. *Why it matters:* coercivity up to ≈ 29 kOe, the magnet grade assumed for sovereign classes and for the demagnetisation margin (FAB-ENV-004; EXOTIC §X2). `pubs.aip.org/aip/adv/article/9/12/125238/1077009/`; `www.sciencedirect.com/science/article/abs/pii/S0304885325000873`

---

## Haptics and physical human–machine interaction

**[R50]** J. E. Colgate and G. G. Schenkel, "Passivity of a class of sampled-data systems: application to haptic interfaces", *J. Robotic Systems* 14(1), 37–47 (1997). *Why it matters:* the passivity condition behind K ≤ 2b/(T + 2T_d), which is why fabrail keeps a little deliberate damping b_dist and cannot render arbitrarily stiff virtual walls (FAB-HAP-009; PHYSICS §17).

**[R51]** D. W. Weir and J. E. Colgate, "Stability of haptic displays", in *Haptic Rendering: Foundations, Algorithms and Applications* (2008). *Why it matters:* the Z-width view of what an admittance device can render, used by the `haptic_zwidth` study (FAB-HAP-009). `web.stanford.edu/class/cs277/resources/papers/Weir2008.pdf`

**[R52]** B. Hannaford and J.-H. Ryu, "Time-domain passivity control of haptic interfaces", *IEEE Trans. Robotics and Automation* 18(1), 1–10 (2002). *Why it matters:* the passivity observer and controller that keep E_obs ≥ −1 mJ (FAB-HAP-009; PHYSICS §17). `www.researchgate.net/publication/3299208_Time-domain_passivity_control_of_haptic_interfaces`

**[R53]** A. Q. L. Keemink, H. van der Kooij and A. H. A. Stienen, "Admittance control for physical human–robot interaction", *Int. J. Robotics Research* 37(11), 1421–1444 (2018). *Why it matters:* the design guidelines behind the admittance law and the limit on rendered-mass ratio: m/M_v ≤ 2 (DRAWER), ≤ 5 sensorless and ≤ 20 with a handle load cell (FAB-HAP-001, FAB-HAP-002). `doi:10.1177/0278364918768950`

**[R54]** N. Hogan, "Impedance control: an approach to manipulation", Parts I–III, *ASME J. Dynamic Systems, Measurement and Control* 107, 1–24 (1985). *Why it matters:* the impedance and admittance framing of touch as an interface (PHYSICS §17).

**[R55]** S. Haddadin, A. Albu-Schäffer and G. Hirzinger, "Requirements for safe robots: measurements, analysis and new insights", *Int. J. Robotics Research* (2009). Not fetched. *Why it matters:* human-impact measurements that support clamping, not free impact, as the governing case for slow, heavy panels (FAB-SAF-005; PHYSICS §22).

**[R56]** S. Haddadin, A. De Luca and A. Albu-Schäffer, "Robot collisions: a survey on detection, isolation, and identification", *IEEE Trans. Robotics* (2017). Not fetched. *Why it matters:* the generalised-momentum observer that estimates hand force from the model residual (FAB-HAP-004, FAB-SAF-006; PHYSICS §17).

**[R57]** Z. Gao, "Scaling and bandwidth-parameterization based controller tuning", *Proc. American Control Conference* (2003). *Why it matters:* bandwidth-parameterised tuning, the pattern behind fabrail's per-class ω_v and observer bandwidths (FAB-PRP-007). `congres.cran.univ-lorraine.fr/2003/ACC%202003/Papers/FP03-3.PDF`

**[R58]** arXiv:2304.14059, a collision-speed bound v_rel,max = F/√(μk) with μ the reduced mass. *Why it matters:* an independent form of the clamp and speed relation used by the critique to cross-check F = v·√(m·k) and the unattended-close limit v ≤ 150 N/√(m_gov·k_series) (FAB-SAF-005, FAB-EXA-006). `arXiv:2304.14059`

---

## Seismology, early warning, ground motion and structural control

**[R59]** R. V. Allen, "Automatic earthquake recognition and timing from single traces", *Bull. Seismol. Soc. Am.* 68(5), 1521–1532 (1978). *Why it matters:* the recursive STA/LTA trigger, reproduced to ±1 sample by the exact discrete recursion (FAB-SEI-002, FAB-SEI-003; PHYSICS §20).

**[R60]** A. Trnkoczy, "Understanding and parameter setting of STA/LTA trigger algorithm", in *New Manual of Seismological Observatory Practice* (NMSOP), GFZ. *Why it matters:* the STA/LTA window, on/off-threshold and LTA-freeze choices (50 / 3000 samples, 4.0 / 1.5) (FAB-SEI-002). `gfzpublic.gfz.de/pubman/faces/ViewItemFullPage.jsp?itemId=item_4097_4`

**[R61]** Y.-M. Wu and H. Kanamori, on-site early warning from the first seconds of the P wave, *Bull. Seismol. Soc. Am.* 95 (2005). Not fetched. *Why it matters:* the τc and Pd parameters and the on-site level matrix (FAB-SEI-004, FAB-SEI-005).

**[R62]** Y.-M. Wu, H. Kanamori, R. M. Allen and E. Hauksson, determination of early-warning parameters τc and Pd for southern California, *Geophys. J. Int.* 170(2), 711 (2007). *Why it matters:* τc and Pd definitions, the 3 s window, the 0.075 Hz high-pass and "Pd > 0.5 cm is damaging" (FAB-SEI-004, FAB-SEI-005). The quoted regression coefficients were inconsistent across fetches, so fabrail uses Wu–Zhao for Pd synthesis. `academic.oup.com/gji/article/170/2/711/849109`

**[R63]** Y.-M. Wu and L. Zhao, "Magnitude estimation using the first three seconds P-wave amplitude in earthquake early warning", *Geophys. Res. Lett.* 33, L16312 (2006). *Why it matters:* log₁₀ Pd = −3.463 + 0.729·M − 1.374·log₁₀ R, which scales the synthetic P phase and gives the anchor Pd = 0.7118 cm for M7 at 20 km (FAB-SEI-004, FAB-SEI-012; PHYSICS §19, §20). `www.semanticscholar.org/paper/Magnitude-estimation-using-the-first-three-seconds-Wu-Zhao/5aa369d5e92f11070ff756a398963025ba987020`

**[R64]** P-Alert low-cost MEMS early-warning network (Taiwan), open-access review. *Why it matters:* MEMS sensing at 100 Hz, ±2 g, and the alert rule Pd ≥ 0.35 cm or PGA ≥ 80 gal adopted in fabrail's PROTECT rule; 2–8 s lead times in practice (FAB-SEI-005). `pmc.ncbi.nlm.nih.gov/articles/PMC8622038/`

**[R65]** USGS, *The blind zone in earthquake early warning*. *Why it matters:* a network blind zone of ≈ 25 km and its latency budget, the reason fabrail never claims warning close to the epicentre (FAB-SEI-012; SEISMIC.md). `www.usgs.gov/programs/earthquake-hazards/science/blind-zone-earthquake-early-warning`

**[R66]** USGS, *Earthquake early warning overview* (ShakeAlert). *Why it matters:* the P-before-S principle and automated actions in practice (FAB-SEI-007). `www.usgs.gov/programs/earthquake-hazards/science/earthquake-early-warning-overview`

**[R67]** ShakeAlert, *Technical partner resources*. *Why it matters:* automated actions need a licensed technical partner, so fabrail ships an alert interface and a synthetic injector only, never a live feed (FAB-SEI-007; SPECIFICATION §5). `www.shakealert.org/technical-partner-resources/`

**[R68]** U.S. Congressional Research Service, R47121, on earthquake early warning. *Why it matters:* automated-action precedents (fire-station doors, elevators) and the first Ridgecrest alert at 6.9 s after origin, context for the ≈ 8 s network-alert assumption behind the 40.8 km crossover (FAB-SEI-012). `www.congress.gov/crs-product/R47121`

**[R69]** Wireless Emergency Alert delivery latency study, *Safety Science* (2022). *Why it matters:* median phone delivery of 6–12 s, which is why fabrail treats network alerts as machine-to-machine inputs and does not rely on human notification (FAB-SEI-007). `www.sciencedirect.com/science/article/pii/S0925753522002375`

**[R70]** JMA earthquake early warning (IPF and PLUM methods), *Frontiers in Earth Science* (2021), and the Wikipedia overview of Japan's system (secondary). *Why it matters:* warning and forecast thresholds and the PLUM approach reused by the exotic fusion (FAB-SEI-007, FAB-EXO-803). `www.frontiersin.org/journals/earth-science/articles/10.3389/feart.2021.726045/full`; `en.wikipedia.org/wiki/Earthquake_Early_Warning_(Japan)`

**[R71]** Government of Japan, *Highlighting Japan* (March 2026), on the elevator P-wave stop requirement. *Why it matters:* a regulatory precedent for automated P-wave actions (stop at the nearest floor and open), the same pattern as FREEZE then land (FAB-SEI-009, FAB-SEI-010). `www.gov-online.go.jp/hlj/en/march_2026/march_2026-09.html`

**[R72]** GeoShake, "How earthquake early warning works" (secondary). *Why it matters:* a plain-language statement of the lead-time principle and elevator P-wave practice, used in the narrative only. `geoshake.org/blog/how-earthquake-early-warning-works`

**[R73]** K. Kanai, semi-empirical formula for the seismic characteristics of the ground, *Bull. Earthquake Res. Inst.* 35 (1957); H. Tajimi, a statistical method of determining the maximum response of a building structure during an earthquake, *Proc. 2nd World Conf. Earthquake Eng.* (1960). Not fetched. *Why it matters:* the Kanai–Tajimi filtered-noise spectrum of the synthetic ground motion (ω_g 10 rad/s, ζ 0.4) (FAB-SEI-018; PHYSICS §19).

**[R74]** R. W. Clough and J. Penzien, *Dynamics of Structures* (McGraw-Hill). *Why it matters:* the Clough–Penzien high-pass correction (ω_f = 0.1·ω_g) that removes the Kanai–Tajimi spectrum's unbounded displacement, and the shear-building model of the Structure tier (FAB-SEI-018, FAB-SEI-021; PHYSICS §19, §21).

**[R75]** P. C. Jennings, G. W. Housner and N. C. Tsai, *Simulated earthquake motions*, Caltech EERL report (1968). Not fetched. *Why it matters:* the JHT amplitude envelope (2, 12, 0.2) applied to the synthetic records (FAB-SEI-018; PHYSICS §19).

**[R76]** M. Shinozuka and G. Deodatis, "Simulation of stochastic processes by spectral representation", *Applied Mechanics Reviews* 44(4), 191–204 (1991). *Why it matters:* the spectral-representation synthesis of seed-deterministic ground motion (FAB-SEI-018; PHYSICS §19).

**[R77]** D. M. Boore, "Simulation of ground motion using the stochastic method", *Pure and Applied Geophysics* 160, 635–676 (2003). *Why it matters:* the stochastic P+S synthesis (`pwave.boore_ps`) whose P phase is scaled so Pd follows Wu–Zhao at the hypocentral distance (FAB-SEI-004, FAB-SEI-010; PHYSICS §19).

**[R78]** J. N. Brune, "Tectonic stress and the spectra of seismic shear waves from earthquakes", *J. Geophys. Res.* 75(26), 4997–5009 (1970). *Why it matters:* the ω² source spectrum with Δσ = 30 bar inside the Boore synthesis (PHYSICS §19).

**[R79]** S. Rezaeian, stochastic modelling and simulation of ground motions (UC Berkeley, eScholarship). *Why it matters:* a fitted non-stationary stochastic model considered as an alternative to the JHT envelope (PHYSICS §19). `escholarship.org/uc/item/51m3669m`

**[R80]** J. P. Den Hartog, *Mechanical Vibrations*, 4th ed. (McGraw-Hill, 1956). *Why it matters:* optimum absorber tuning f = 1/(1 + μ) and ζ = √(3μ/(8(1 + μ)³)), the anchors 0.98039 and 0.08407 at μ = 0.02 (FAB-SEI-016, FAB-EXO-601; PHYSICS §21, EXOTIC §X6).

**[R81]** G. B. Warburton, "Optimum absorber parameters for various combinations of response and excitation parameters", *Earthquake Engineering and Structural Dynamics* 10(3), 381–401 (1982). *Why it matters:* the base-acceleration and force (wind) optima, anchors 0.97548 / 0.07019 at μ = 0.02 (FAB-SEI-016, FAB-EXO-601). `doi:10.1002/eqe.4290100304`

**[R82]** B. F. Spencer Jr., S. J. Dyke and H. S. Deoskar, benchmark problems in structural control, part I: active mass driver system, *Earthquake Engineering and Structural Dynamics* 27 (1998). *Why it matters:* an AMD benchmark with explicit force and stroke limits, the model for honouring limits in the closed-loop Structure study (FAB-SEI-021). `sstl.cee.illinois.edu/benchmarks/bench1journal/BENCHamd.pdf`

**[R83]** "Active mass driver system as the first application of active structural control" (Kyobashi Seiwa building, 1989), with the Kobori et al. figure source. *Why it matters:* the first full-scale AMD used ≈ 1 % of the structure's mass (4 t + 1 t, T = 1.13 s). A 5 t fabrail panel in a 10⁷ kg building is 0.05 %, which is why AMD by a rail panel is research tier (FAB-SEI-016, FAB-SEI-021). `www.researchgate.net/publication/229569268_Active_mass_driver_system_as_the_first_application_of_active_structural_control`; `www.researchgate.net/figure/The-Kyobashi-Seiwa-Building-Source-Kobori-T-et-al-Seismic-Response-Controlled_fig2_242397505`

**[R84]** B. F. Spencer Jr., review of active and hybrid structural control in practice, UJNR joint panel paper (PWRI). *Why it matters:* the installed base of active and hybrid dampers, most of them sized for wind comfort, context for SEISMIC.md reading (b) and SENTINEL (FAB-EXO-607). `www.pwri.go.jp/eng/ujnr/joint/34/paper/22spence.pdf`

**[R85]** S. M. Mousavi et al., STEAD: STanford EArthquake Dataset (2019), CC BY 4.0. *Why it matters:* labelled P and S picks available to the `detect` command for detector evaluation (SEISMIC.md; FAB-SEI-002). `github.com/smousavi05/STEAD`

**[R86]** CESMD (Center for Engineering Strong Motion Data), data policy and the El Centro 1940 record (operated by USC&GS, 0.359 g). *Why it matters:* the attribution terms for El Centro when a user fetches it on request; fabrail ships no copy (SEISMIC.md). `www.strongmotioncenter.org/NCESMD/reports/CESMD_DataPolicy.htm`; `www.strongmotioncenter.org/cgi-bin/CESMD/iqr_dist_DM2.pl?IQRID=ElCentro_18May1940&SFlag=0&Flag=3`

**[R87]** NOAA NCEI, digitised accelerograms (federal collection, decommissioned 2025). *Why it matters:* a second, public-domain source path for historical records (SEISMIC.md). `www.ncei.noaa.gov/access/metadata/landing-page/bin/iso?id=gov.noaa.ngdc.mgg.hazards%3AG01145`

**[R88]** OpenSees verification example `elCentro.at2`. *Why it matters:* the file named "elCentro" is really the 1968 Borrego Mountain record, a provenance trap SEISMIC.md warns about. `github.com/DBorello/OpenSees/blob/master/EXAMPLES/verification/elCentro.at2`

**[R89]** PEER NGA-West2 database terms of use. *Why it matters:* copyright, download limits and acknowledgement terms that keep NGA records out of the repository. `ngawest2.berkeley.edu/`

**[R90]** New Atlas, "Air Danshin" air-lift base isolation for houses (secondary). *Why it matters:* a non-magnetic precedent that lifts a house about 3 cm within 0.5–1 s on a seismic trigger, a benchmark for what an alert-triggered lift must achieve and a reminder that lift-on-alert was dropped from fabrail. `newatlas.com/airlift-system-levitate-houses-earthquakes/21762/`

**[R91]** Arx Pax "hover foundation" (Magnetic Field Architecture), read through Global Construction Review and Patent Yogi (secondary). *Why it matters:* a cautionary tale for whole-building magnetic levitation, which fabrail never claims (SPECIFICATION §5). `www.globalconstructionreview.com/hoverboard-company-pate8nts-e4arth0qu6ak4e2-p0r8oo/`; `patentyogi.com/arx-pax/technology-magnetically-levitates-buildings-earthquakes-make-hover-shaky-ground/`

**[R92]** J. Paul Getty Museum, the engineering of artifact isolators (steel frame, rollers and springs, up to ≈ 90 % acceleration reduction). *Why it matters:* the passive baseline that HAVEN-R must beat to justify maglev (FAB-EXO-209, FAB-EXO-505). `www.getty.edu/news/the-hidden-engineering-protecting-getty-art-from-earthquakes/`

**[R93]** A. G. Davenport, "The spectrum of horizontal gustiness near the ground in high winds", *Q. J. Royal Meteorological Society* 87 (1961). Not fetched. *Why it matters:* the wind-load spectrum for SENTINEL's wind-day scenario (FAB-EXO-607; EXOTIC §X15).

---

## Prior art: products and systems

**[R94]** Transrapid (Wikipedia) and a report on its long-stator LSM sections (OSTI ETDE). *Why it matters:* 10 mm EMS gap and long-stator iron-core LSM sections of 500–2000 m. Levitation power of ≈ 1–2 kW/t [UNCERTAIN] sets the pure-EMS hover-loss scale that zero-power hybrid suspension avoids (FAB-EXO-206; PHYSICS §2, §7). `en.wikipedia.org/wiki/Transrapid`; `www.osti.gov/etdeweb/biblio/506976`

**[R95]** SCMaglev (Wikipedia; JR Central). *Why it matters:* 100 mm electrodynamic suspension, the contrast that shows why speed-dependent EDS cannot hold a drawer at rest. `en.wikipedia.org/wiki/SCMaglev`; `scmaglev.jr-central-global.com/about/`

**[R96]** R. F. Post, Inductrack (OSTI report) and the Wikipedia summary. *Why it matters:* passive Halbach EDS with a transition speed near walking speed; formulas quoted from memory [UNCERTAIN]. It is not a rest-state suspension, so fabrail uses EMS. `www.osti.gov/biblio/791522`; `en.wikipedia.org/wiki/Inductrack`

**[R97]** Beckhoff XPlanar planar motor, and an XPlanar-class 6-DOF control paper (arXiv:2509.01388). *Why it matters:* a commercial 6-DOF maglev reference: Hall arrays, Halbach movers, ≈ 5 mm lift, µm repeatability and 4 kHz (250 µs) control, against fabrail's 20 kHz / 10 kHz groups (FAB-FW-004). `www.beckhoff.com/en-us/products/motion/xplanar-planar-motor-system/`; `arXiv:2509.01388`

**[R98]** Beckhoff XTS linear transport (product page, news release) and a DMC summary (secondary). *Why it matters:* 75 / 168 / 210 N mover forces, 4 m/s and 10 µm, the industrial LSM envelope fabrail's classes sit below. `www.beckhoff.com/en-us/products/motion/xts-linear-product-transport/`; `www.beckhoff.com/en-en/company/news/multimedia-the-perfectly-synchronized-xts-dynamic-precise-scalable.html`; `www.dmcinfo.com/blog/18332/beckhoff-xplanar-xts/`

**[R99]** Planar Motor XBot 4-series, and a Magnetics Magazine report (secondary). *Why it matters:* ≤ 6 mm levitation for 1.8–40 kg movers at 54 V with 6 W to 1500 W draw, a comparison point for the DRAWER's 8–48 kg range and hover budget (FAB-PWR-002). `www.planarmotor.com/4-series`; `magneticsmag.com/seeing-planar-motor-incs-magnetic-levitation-system-in-action-is-believing/`

**[R100]** MagneMotion QuickStick 100 manual and QuickStick HT technical data (Rockwell). *Why it matters:* 2.5 m/s, 1 g, 100 kg movers and 182 N per cycle at a 12 mm gap, the long-stator sizing reference for WALL and HEAVY thrust (FAB-PRP-003). `www.manualslib.com/manual/1474582/Magnemotion-Quickstick-100.html?page=229`; `literature.rockwellautomation.com/idc/groups/literature/documents/td/mmi-td054_-en-p.pdf`

**[R101]** Rockwell iTRAK 5750. *Why it matters:* 5 m/s movers with safety functions rated by the vendor to SIL 3 / PL e: safety-rated LSM functions exist in industry, and fabrail claims no such rating. `www.rockwellautomation.com/en-us/products/hardware/motion-control/itrak-5750.html`

**[R102]** TK Elevator MULTI (Wikipedia, two articles) and a *Transportation Systems and Technology* article. *Why it matters:* a building-scale LSM with plastic glider guidance, sensing, brakes and an energy buffer, the closest prior art for long-stator propulsion inside buildings. `en.wikipedia.org/wiki/TK_Elevator_MULTI`; `en.wikipedia.org/wiki/Thyssenkrupp_MULTI`; `journals.eco-vector.com/transsyst/article/view/10429/en_US`

**[R103]** dormakaba CS 80 MAGNEO (product pages). *Why it matters:* a commercial linear-motor sliding door (80 kg, SoftMotion, Push & Go, low-energy without extra sensors), the DOOR class's nearest product. `www.dormakaba.com/gb-en/offering/products/entrance-systems/sliding-door-operators/cs-80-magneo--do_166`; `www.dormakaba.com/ch-en/offering/products/entrance-systems/sliding-door-operators/automatic-sliding-door-operator-for-interior-doors-cs-80-magneo-1-leaf--do_166`; `www.dormakaba.com/us-en/solutions/products/entrance-systems/automatic-sliding-doors---icu-doors-/magneo-292408`

**[R104]** GEZE ECdrive T2. *Why it matters:* 140 kg (200 kg Powerdrive) automatic sliding operators, the DOOR mass range. `www.geze.com/en/products-solutions/sliding_doors/automatic_sliding_doors/ecdrive/ecdrive_t2/p_125478`

**[R105]** Häfele / Hawa SoftMove 80. *Why it matters:* the hydraulic soft-close damper that fabrail's soft-close and log-damped terminus replace in software (FAB-HAP-007, FAB-HAP-013). `www.hafele.com/us/en/product/soft-and-self-closing-mechanism-softmove-80-for-hawa-junior-80/P-00876696/`

**[R106]** dormakaba MUTO with DORMOTION. *Why it matters:* 50–150 kg manual sliding doors with mechanical soft-close, the manual baseline the DOOR's rendered feel is compared with. `www.dormakaba.com/us-en/offering/products/interior-glass-systems/manual-sliding-door-systems`

**[R107]** Blum SERVO-DRIVE: a knock-to-open integration note (secondary) and an FCC filing. *Why it matters:* about 2 mm trigger travel and a 24 V / 2 A drive with 0.12–0.2 W standby, the benchmarks for push-to-open (1.5 mm) and the seated sleep budget (≤ 0.3 W) (FAB-HAP-012, FAB-PWR-002). `industrialmonitordirect.com/blogs/knowledgebase/blum-servo-drive-knock-to-open-integration-solenoid-trigger`; `fcc.report/FCC-ID/w9523p502l00/4439981.pdf`

**[R108]** Blum TIP-ON BLUMOTION. *Why it matters:* a 2.5 mm front gap and load classes for mechanical push-to-open drawers, the envelope the DRAWER seat's x push-stop compliance (≥ 2 mm) mirrors (FAB-LEV-017). `www.blum.com/us/en/products/motion-technologies/tip-on-blumotion/overview/`

**[R109]** Skyfold operable walls (product site and ARCAT specification). *Why it matters:* 1.5–3 m/min, STC 51–60, 6.0–9.4 lb/ft², the speed and mass context for the WALL class. `skyfold.com/en-US`; `www.arcat.com/sdspecs/htm04/10_22_39ski.htm`

**[R110]** Modernfold Encore and Acousti-Seal Legacy panels. *Why it matters:* 30 ft/min and STC 56, 6–11 lb/ft², the ballroom-partition reference behind the WALL preset. `www.modernfoldstyles.com/products/modernfold-encore-panels/`; `www.archdaily.com/catalog/en/products/39599/single-panel-operable-wall-acousti-seal-legacy-modernfold`

**[R111]** Brombal Levitate, IronLev and a Core77 report (secondary). *Why it matters:* passive permanent-magnet sliding doors of 900–1000 kg at μ ≈ 0.004 with an 8 mm brushed slot. Passive magnets alone cannot be stable in every axis, so some axis is held mechanically; these products set the friction benchmark that fabrail's modelled μ_eq ≈ 1e-3 to 1e-2 is compared with (FAB-HAP-010). `discoverbrombal.com/levitate/`; `www.ironlev.com/`; `www.core77.com/posts/116528/A-Mag-Lev-System-for-Massive-Sliding-Doors`

**[R112]** Passive NdFeB maglev vibration isolators for light structures, *Structural Control and Health Monitoring* (2022). *Why it matters:* passive magnetic isolation works at light-structure scale; fabrail's active isolation stays research tier (FAB-SEI-014). `onlinelibrary.wiley.com/doi/full/10.1002/stc.2973`

**[R113]** Hardt Hyperloop progress paper, and a low-confidence secondary review. *Why it matters:* hybrid EMS with more than 1000 corrections per second and a linear synchronous reluctance motor, a vehicle-scale reference for hybrid EMS control rates. `docs.hardt.global/hyperloop-progress-paper`; `www.borntoengineer.com/hyperloop-engineering-reality-check-2025`

**[R114]** Railway Gazette, Nevomo magnetic levitation test (2 t, 135 km/h, lift-off near 70 km/h) (secondary). *Why it matters:* speed-dependent lift in retrofit maglev, the contrast to a rest-capable EMS. `www.railwaygazette.com/traction-and-rolling-stock/nevomo-hails-magnetic-levitation-test-success/64907.article`

**[R115]** TechCrunch, the Hendo hoverboard over copper (secondary). *Why it matters:* eddy-current levitation needs a changing field over a conductor, a technology the research set aside for rails that must hold still. `techcrunch.com/2015/10/21/real-hoverboard/`

**[R116]** Open-source motor-control stacks: mjbots moteus (30 kHz loop), ODrive 0.5.6 (8 kHz cascade), SimpleFOC (`loopFOC()` / `move()` split) and VESC `bldc`. *Why it matters:* the loop-rate and FOC structure precedents for the 20 kHz current group and the fast/supervisor split (PHYSICS §4, §7, §24). `mjbots.github.io/moteus/reference/theory/`; `docs.odriverobotics.com/v/0.5.6/control.html`; `docs.simplefoc.com/motion_control_implementation`; `github.com/vedderb/bldc`

**[R117]** Open-source hobby levitation projects `Matrixchung/Maglev` and `famichu/MagLev_Switch_MX`. *Why it matters:* small-scale single-axis levitation code, useful as a sanity reference for gap-loop structure only. `github.com/Matrixchung/Maglev`; `github.com/famichu/MagLev_Switch_MX`

**[R118]** ethercrab, a `no_std` EtherCAT master in Rust. *Why it matters:* a candidate fieldbus for multi-carriage installations (roadmap). `github.com/ethercrab-rs/ethercrab`

---

## Embedded, firmware and software tooling

**[R119]** RTIC v2 book, the RTIC repository and RTIC PR #888 (the 2.x stable release). *Why it matters:* stack-resource-policy scheduling on hardware interrupts, the executive of the STM32G474 firmware (FAB-FW-001, FAB-FW-007). RTIC supports Cortex-M fully and has no Cortex-R support, so a Cortex-R5F safety channel needs a cyclic executive (EXOTIC §S4). `rtic.rs/2/book/en/`; `github.com/rtic-rs/rtic`; `github.com/rtic-rs/rtic/pull/888`

**[R120]** crates.io metadata for the pinned firmware stack, queried 2026-09-26: rtic 2.3.1, rtic-monotonics 2.2.1, stm32g4xx-hal 0.1.0 (features `stm32g474`, `hrtim`, `can`, `adc3-5`; dependency list, docs.rs build log, ADC source), stm32g4 0.16, cortex-m 0.7.9, cortex-m-rt 0.7.7, defmt 1.1.1, defmt-rtt 1.3.0 and panic-probe 1.0.0; considered and not used: embassy-stm32 0.6.0, stm32-hrtim, fdcan, panic-halt. *Why it matters:* the exact pins that FAB-FW-001 inspects and that the release firmware links against (FAB-FW-010). `crates.io/api/v1/crates/rtic`; `crates.io/api/v1/crates/stm32g4xx-hal/0.1.0/dependencies`; `docs.rs/crate/stm32g4xx-hal/0.1.0/builds/2879586`; `raw.githubusercontent.com/stm32-rs/stm32g4xx-hal/main/src/adc/mod.rs`

**[R121]** STMicroelectronics, STM32G474RE product page, datasheet DS12288 and reference manual RM0440. *Why it matters:* the firmware target's HRTIM, ADC, FDCAN, COMP, CORDIC and FMAC peripherals and interrupt names; the hardware-only latency and cycle items stay roadmap (FAB-FW-001, FAB-FW-012). `www.st.com/en/microcontrollers-microprocessors/stm32g474re.html`; `www.st.com/resource/en/datasheet/stm32g474re.pdf`

**[R122]** Analog Devices, ADXL354/ADXL355 datasheet and product page. *Why it matters:* 22.5 µg/√Hz noise density, ±2 g, 20-bit output and a 500 Hz ODR, the seismic accelerometer model mounted on the structure or rail (FAB-SEI-001, FAB-SEI-006; PHYSICS §13). `www.analog.com/media/en/technical-documentation/data-sheets/adxl354_adxl355.pdf`; `www.analog.com/en/products/adxl355.html`

**[R123]** C. L. Liu and J. W. Layland, "Scheduling algorithms for multiprogramming in a hard-real-time environment", *J. ACM* 20(1), 46–61 (1973). *Why it matters:* the utilisation bound behind the schedulability budget ΣC/T ≤ 0.70 (current sum 0.64) (FAB-FW-008).

**[R124]** PyO3 (0.29; migration guide) and maturin (≥ 1.7). *Why it matters:* the `fabrail._native` extension that carries arrays as little-endian bytes without rust-numpy (FAB-SW-006). `pyo3.rs/main/migration.html`

**[R125]** NumPy Enhancement Proposal 19, *Random number generator policy*. *Why it matters:* numpy streams may change across versions, so the Rust sensor-noise streams are in-repo (SplitMix64 + Xoshiro256++) and the anchors record the numpy version used for scenario synthesis (FAB-SW-002, FAB-SW-003; PHYSICS §24).

**[R126]** NumPy (≥ 2.0), SciPy (≥ 1.10) and Matplotlib (≥ 3.7). *Why it matters:* scenario synthesis, the reference detector, offline LQR gains (scipy CARE) and every figure (FAB-SEI-002, FAB-SW-009).

**[R127]** The Rust `libm` crate 0.2.16 with `force-soft-floats`. *Why it matters:* every controller transcendental goes through `fabrail_types::fmath` on libm, so native, wasm32 and thumbv7em builds compute the same bits and the golden selftest hash can match (FAB-FW-002, FAB-FW-005, FAB-SW-014; PHYSICS §24, §26).

**[R128]** proptest 1.10, Hypothesis and pytest. *Why it matters:* the property tests (integer monitor, allocation, 1 kg–1e8 kg exotic units) and the marker-based traceability that FAB-SW-007 checks (FAB-SAF-012, FAB-EXO-112).

**[R129]** rayon 1.12, serde 1.0.228 and serde_json with `float_roundtrip`. *Why it matters:* worker-count-invariant ensembles and bit-exact JSON round trips of parameters and scenarios (FAB-SW-002, FAB-SW-015).

**[R130]** The Rust toolchain via rustup with the thumbv7em-none-eabihf, thumbv6m-none-eabi, armv7r-none-eabihf and wasm32-unknown-unknown targets. *Why it matters:* real cross builds of the control and safety crates and the browser engine are gates, not claims of hardware execution (FAB-FW-013, FAB-FW-010, FAB-SW-014).

**[R131]** Ferrous Systems: Ferrocene libcore certification news (SIL 2 libcore, SIL 3 toolchain), the Ferrocene 26.05 release and the Ferrocene target list. *Why it matters:* a qualified toolchain does not certify the application built with it, stated in EXOTIC §S4 (FAB-EXO-905). `ferrous-systems.com/blog/ferrocene-libcore-news-release/`; `ferrous-systems.com/blog/ferrocene-26-05-0/`; `public-docs.ferrocene.dev/main/user-manual/targets/index.html`

**[R132]** Ferrous Systems, Rust on Cortex-R52, and the rust-embedded `aarch32` crates (aarch32-cpu, aarch32-rt). *Why it matters:* the runtime path for an Armv7-R/Armv8-R safety channel without RTIC (EXOTIC §S4). `ferrous-systems.com/blog/rust-cortex-r52/`; `github.com/rust-embedded/aarch32`

**[R133]** HighTec Rust development platform, and New Electronics on its ISO 26262 ASIL D-qualified compiler for Infineon AURIX. *Why it matters:* evidence that qualified Rust compilers exist for lockstep automotive MCUs; not used by fabrail (EXOTIC §S4). `hightec-rt.com/products/rust-development-platform`; `www.newelectronics.co.uk/content/news/hightec-unveils-iso-26262-asil-d-qualified-rust-compiler-for-infineon-aurix-mcus`

**[R134]** Texas Instruments TMS570LC4357, and a TI E2E thread on its BE32 endianness. *Why it matters:* a SIL 3-capable lockstep part that current Rust targets cannot build for, stated as untargetable (FAB-EXO-905). `www.ti.com/product/TMS570LC4357`; `e2e.ti.com/support/tools/code-composer-studio-group/ccs/f/code-composer-studio-forum/1071475/in-ccs-11-1-a-project-for-a-tms570lc4357-using-the-ti-clang-compiler-no-longer-supports-the-correct-device-endianess-of-be32`

**[R135]** Texas Instruments AM2634. *Why it matters:* 4× Cortex-R5F at 400 MHz with lockstep clusters, the armv7r reference part for the sovereign safety channel and its WCET budget (FAB-EXO-904). `www.ti.com/product/AM2634`

**[R136]** NXP S32K3 family. *Why it matters:* lockstep Cortex-M7 parts, an alternative safety-channel MCU. One lockstep chip is SIL 3 at most (EXOTIC §S4). `www.nxp.com/products/processors-and-microcontrollers/s32-automotive-platform/s32k-auto-general-purpose-mcus/s32k3-microcontrollers-for-automotive-general-purpose:S32K3`

---

## Sovereign tier: reference installations, dampers and storage

These sources back the sovereign-tier classes of EXOTIC.md. None of them makes a fabrail class a product.

**[R137]** ESO, Extremely Large Telescope dome. *Why it matters:* a 6,100 t rotating dome on 36 stationary trolleys, the ROTATOR reference (EXOTIC §1). `elt.eso.org/telescope/dome/`

**[R138]** Uni-Systems, State Farm Stadium field tray. *Why it matters:* an 8,600 t tray on 542 wheel assemblies moving at ≈ 0.042 m/s, the ARENA reference (FAB-EXO-101). `www.uni-engineer.com/portfolio/state-farm-stadium-glendale-arizona/`

**[R139]** Kawasaki Heavy Industries, Sapporo Dome hovering stage. *Why it matters:* an 8,300 t air-lifted (7.5 cm, −90 % weight) field that moves at 4 m/min, the non-magnetic precedent for moving stadium-scale mass. `global.kawasaki.com/en/corp/newsroom/news/detail/ba010801.html`

**[R140]** Maeslantkering (Wikipedia). *Why it matters:* two 6,800 t arms on rail "locomobiles" with a decision system that closes on a forecast, the precedent for TIDEGATE's external decision path (FAB-EXO-809). `en.wikipedia.org/wiki/Maeslantkering`

**[R141]** MOSE, Venice (Wikipedia). *Why it matters:* buoyant flap gates, an example of hydraulic and buoyant gates beating maglev physically (EXOTIC §1). `en.wikipedia.org/wiki/MOSE`

**[R142]** Thames Barrier (Wikipedia). *Why it matters:* 3,700 t rotary sector gates closing in about 10 min each, a closure-time benchmark for TIDEGATE. `en.wikipedia.org/wiki/Thames_Barrier`

**[R143]** Japanese tsunami gates (e.g. Fudai, Iwate) and a *Hitachi Review* (2021) article on automatic and remote gate closure. *Why it matters:* post-2011 automatic closure on a JMA intensity or J-ALERT trigger, the trigger stack TIDEGATE copies (FAB-EXO-809). `iwatetabi.jp/en/spots/99420/`; `www.hitachihyoron.com/rev/archive/2021/r2021_03/07/index.html`

**[R144]** Taipei 101 tuned mass damper (Wikipedia; Guinness World Records; Taipei 101 observatory). *Why it matters:* a 660 t pendulum TMD with viscous dampers and 1.0 m of swing in a 2015 typhoon: the scale SENTINEL is sized against, and a wind device first (FAB-EXO-607). `en.wikipedia.org/wiki/Taipei_101`; `www.guinnessworldrecords.com/world-records/774616-largest-tuned-mass-damper`; `www.taipei-101.com.tw/en/observatory/feature`

**[R145]** Citigroup Center TMD (Wikipedia). *Why it matters:* a 360 t concrete block sliding on oil-coated steel plates, the friction-bearing TMD whose activation threshold maglev could remove (FAB-EXO-609). `en.wikipedia.org/wiki/Citigroup_Center`

**[R146]** Mitsubishi Heavy Industries, Tokyo Skytree TMDs. *Why it matters:* 40 t + 25 t passive inverted-pendulum dampers with ±125 cm capacity, a stroke benchmark for SENTINEL's ±1.5 m (FAB-EXO-110). `www.mhi.com/news/1102011403.html`

**[R147]** Shanghai Tower eddy-current TMD: Motioneering, a Gizmodo article (secondary) and Lu et al. (2017) in *Structural Control and Health Monitoring*. *Why it matters:* a 1,000 t pendulum damped by NdFeB magnets over copper with no external power, the passive eddy damping SENTINEL keeps as its fallback (FAB-EXO-605). `www.motioneering.ca/shanghai-tower`; `gizmodo.com/how-a-121-storey-building-uses-a-giant-magnet-to-preven-1692540759`; `onlinelibrary.wiley.com/doi/abs/10.1002/stc.1882`

**[R148]** Shanghai World Financial Center active TMD (Mori Building; MHI Australia). *Why it matters:* an active damper installed against wind sway, the active-mode precedent for SENTINEL (FAB-EXO-607). `www.mori.co.jp/en/press/release/antiwind_technology_to_be_inst/`; `www.mhiaa.com.au/the-first-tuned-active-damper-in-mainland-china-designed-to-prevent-a-high-rise-building-from-wind-sway/`

**[R149]** Yokohama Landmark Tower tuned active damper (JSME). *Why it matters:* a hybrid active damper, precedent for the DVF-only and passive fallbacks in SENTINEL's mode ladder (FAB-EXO-605). `www.jstage.jst.go.jp/article/jsmec1993/37/3/37_3_450/_article/-char/en`

**[R150]** Tuned mass damper (Wikipedia), including 432 Park Avenue and 111 West 57th Street (masses [UNCERTAIN]). *Why it matters:* supertall residential dampers of 600–730 t, the billionaire-scale SENTINEL context. `en.wikipedia.org/wiki/Tuned_mass_damper`

**[R151]** TMD friction studies: *Engineering Structures* (2022) and Motioneering, "Damping explained". *Why it matters:* friction lock-out is a documented TMD failure mode, and the industry rule is that friction must be overcome at one tenth of the target acceleration. Maglev removes that threshold only on low-coercivity or homopolar rails (FAB-EXO-609; EXOTIC §X8). `www.sciencedirect.com/science/article/abs/pii/S0141029622006034`; `www.motioneering.ca/post/damping-explained-impact-friction-tuned-mass-dampers`

**[R152]** J. J. Connor, *Introduction to Structural Motion Control*, chapter 4 (Citicorp, Chiba and Crystal Tower dampers). *Why it matters:* the textbook design of tuned and active mass dampers and their stroke limits, used for the ζ_add ≤ (μ/2)(S/X) bound (FAB-EXO-603; EXOTIC §X6). `engineering.purdue.edu/~ce573/Documents/Intro%20to%20Structural%20Motion%20Control_Chapter4.pdf`

**[R153]** Three-dimensional seismic isolation R&D for a Japanese fast-breeder reactor building (OSTI ETDE). *Why it matters:* ±1.0 m horizontal and ±0.5 m vertical isolation on rolling-seal air springs, the evidence that vertical isolation at scale is passive, which HAVEN keeps (FAB-EXO-705). `www.osti.gov/etdeweb/biblio/20814171`

**[R154]** CERN, CMS heavy lowering. *Why it matters:* 2,000 t detector elements moved on air pads with hydraulic jacks, the non-magnetic precedent for SHIELD-class moves. `cms.cern/detector/heavy-lowering`

**[R155]** Cyclife Aquila, ESS concrete shielding doors. *Why it matters:* 19 t shield doors on four wheel units with a manual chain drive, the SHIELD baseline. `cyclifeaquila.com/projects/concrete-shielding-doors/`

**[R156]** Veritas SmartDoor radiation-shielded doors, and U.S. patent 9,458,661. *Why it matters:* radiotherapy vault doors of about 5 t, the SHIELD entry point. `www.veritas-medicalsolutions.com/smartdoor-radiation-shielded-door/`; `image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9458661`

**[R157]** The Shed, Hudson Yards (Wikipedia). *Why it matters:* a ≈ 3,630 t moving shell on 1.8 m bogie wheels, an ARENA-scale moving-building precedent. `en.wikipedia.org/wiki/The_Shed_(Hudson_Yards)`

**[R158]** Stade de France retractable lower tier (Wikipedia). *Why it matters:* 10 elements of 700 t moved over 80 h by 40 people, the labour cost ARENA's pods would remove. `en.wikipedia.org/wiki/Stade_de_France`

**[R159]** Bank vault (Wikipedia) and the NORAD Cheyenne Mountain blast doors (Wikipedia; a secondary summary). *Why it matters:* 20–25 t hinged vault and blast doors that close in tens of seconds, the BASTION baseline (FAB-EXO-807). `en.wikipedia.org/wiki/Bank_vault`; `en.wikipedia.org/wiki/Cheyenne_Mountain_Complex`; `citystatezipcode.com/facts-hidden-behind-cheyenne-mountains-25-ton-blast-doors/`

**[R160]** Suite Vollard and the Dynamic Tower concept (Wikipedia). *Why it matters:* rotating floors at about one revolution per hour, the ROTATOR residential variant. `en.wikipedia.org/wiki/Suite_Vollard`; `en.wikipedia.org/wiki/Dynamic_Tower`

**[R161]** Three Gorges ship lift (infra.global; National Geographic, secondary). *Why it matters:* a 15,500 t counterweighted caisson on rack and pinion, the vertical-lift scale reference. `infra.global/projects/three-gorges-ship-lift/`; `www.nationalgeographic.com/magazine/article/explore-china-three-gorges-dam-ship-lift`

**[R162]** Mercedes-Benz Stadium (Wikipedia) and *Stadia* magazine on the roof becoming operational. *Why it matters:* the opening was delayed by the complexity of the eight-panel retractable roof, each panel on two rails. The ARENA narrative says only that, not that a specific failure mode occurred. `en.wikipedia.org/wiki/Mercedes-Benz_Stadium`; `www.stadia-magazine.com/news/roofing-materials-technologies/mercedes-benz-stadium-roof-finally-operational.html`

**[R163]** Tottenham Hotspur, the dividing retractable pitch (club news, secondary). *Why it matters:* three trays of more than 3,000 t on wheels at 25 t each, the pod-count logic behind ARENA's static reallocation (FAB-EXO-101). `www.tottenhamhotspur.com/news-archive-1/scx-to-supply-world-first-dividing-retractable-pitch-for-tottenham-hotspur-s-new-stadium/`

**[R164]** Ampelmann A-type motion-compensated gangway. *Why it matters:* a 39 t hydraulic Stewart platform carrying people, the occupied-platform precedent for HAVEN's OccupiedPlatform regime. `www.technologycatalogue.com/product_service/ampelmann-type`

**[R165]** NASA Vehicle Assembly Building (Wikipedia). *Why it matters:* 139 m high-bay doors that take about 45 minutes, the slow-door end of the sovereign range. `en.wikipedia.org/wiki/Vehicle_Assembly_Building`

**[R166]** Electromagnetic Aircraft Launch System (Wikipedia). *Why it matters:* 121 MJ delivered per launch from about 357 MJ stored, and a reliability shortfall against its requirement: the storage anchor and the caution for ride-through sizing (FAB-EXO-403). `en.wikipedia.org/wiki/Electromagnetic_Aircraft_Launch_System`

**[R167]** Flywheel energy storage (Wikipedia). *Why it matters:* energy densities and the JET flywheel generators (775 t, 3.75 GJ, 400 MW), the "one JET-class unit" yardstick against which pure-EMS ride-through is flagged insufficient (FAB-EXO-401, FAB-EXO-403). `en.wikipedia.org/wiki/Flywheel_energy_storage`

**[R168]** Superconducting magnetic energy storage (Wikipedia). *Why it matters:* 1–10 Wh/kg with millisecond response; documented only, not modelled (FAB-EXO-401). `en.wikipedia.org/wiki/Superconducting_magnetic_energy_storage`

**[R169]** SKF S2M active magnetic bearings (bearing-news; Turbomachinery Magazine). *Why it matters:* industrial AMBs carry more than 30 t of axial load on hydro-turbine shafts, evidence that tonne-scale active magnetic suspension is routine in rotating machinery. `www.bearing-news.com/skf-active-magnetic-bearings/`; `www.turbomachinerymag.com/view/10039-2`

**[R170]** Magnetic load reduction for hydro generators: Ma (2013) in *Int. J. Applied Electromagnetics and Mechanics*, a 2015 study, the Vattenfall Porjus feasibility study (Vinnova) and a River Publishers *ACES* paper. *Why it matters:* hybrid PM + EM devices already take part of a rotor's weight (example axial load 1,387 t), the closest industrial analogue of hybrid EMS at the kiloton scale (EXOTIC §X4). `journals.sagepub.com/doi/10.3233/JAE-131695`; `onlinelibrary.wiley.com/doi/10.1155/2015/814084`; `www.vinnova.se/en/p/power-increasing-magnetic-bearing-for-hydropower-generators-part-2/`; `journals.riverpublishers.com/index.php/ACES/article/download/9535/7979/30639`

---

## Sovereign tier: superconducting levitation, warning networks, blast and tsunami

**[R171]** U. Floegel-Delor et al., HTS bulk maglev cryostats, *IEEE Trans. Applied Superconductivity* 29(5), 3601705 (2019). *Why it matters:* the 300 kgf at 20 mm and 550 kgf at 15 mm points the HTS model is fitted to, and the LN₂ hold times (FAB-EXO-202, FAB-EXO-204; EXOTIC §X3). `www.superconductor-company.com/fileadmin/user_upload/4-U-Floegel-Delor-et-al-2019-IEEE-TAS-29-5-3601705.pdf`

**[R172]** HTS H-formulation model parameters, *Materials* (2019). *Why it matters:* Jc(T) = Jc0·[(1 − (T/Tc)²)/(1 − (77/Tc)²)]² with Jc0 1.1e8 A/m² at 77 K and Tc 92 K, the temperature law of the reduced-order HTS model (FAB-EXO-204). `pmc.ncbi.nlm.nih.gov/articles/PMC6766180/`

**[R173]** L. Quéval et al., H-formulation FEM of superconducting magnetic bearings, validated for field-cooled and zero-field-cooled motion. *Why it matters:* the high-fidelity reference the reduced-order HTS model is simpler than (EXOTIC §X3). `arXiv:1803.06741`

**[R174]** The frozen-image model of HTS levitation. *Why it matters:* the conservative core of fabrail's HTS force model, before play-operator hysteresis is added (FAB-EXO-203). `arXiv:cond-mat/0111316`

**[R175]** Flux locking in HTS levitation, *Physica C* (2000). *Why it matters:* the physics of lateral pinning, which is why HTS is allowed only in 1-D stages whose free axis is the travel axis (EXOTIC §X3). `www.sciencedirect.com/science/article/abs/pii/S0921453400008170`

**[R176]** J. H. Durrell et al., 17.6 T trapped in bulk GdBCO at 26 K, *Superconductor Science and Technology* 27, 082001 (2014). *Why it matters:* the trapped-field record behind FAB-FRN-002, where pressure is still capped by rail saturation. `iopscience.iop.org/article/10.1088/0953-2048/27/8/082001`

**[R177]** Active flux-pumped superconducting levitation. *Why it matters:* a height-tunable, creep-cancelling alternative to passive HTS, recorded as research (EXOTIC §X3). `arXiv:2403.19168`

**[R178]** HTS levitation after cryogen depletion or cryocooler stop. *Why it matters:* slow warm-up, the 4 % critical LN₂ content and a fade of tens of minutes, which fabrail models instead of an hours-long fade (FAB-EXO-205). `arXiv:2501.09030`

**[R179]** Levitation force relaxation in YBCO superconductors (ResearchGate). *Why it matters:* logarithmic creep sc(t) = 1 − S·log₁₀(1 + t/τ0), up to 40 % in bad cases, modelled as quasi-static only (FAB-EXO-204, FAB-EXO-208). `www.researchgate.net/publication/238547954_Levitation_force_relaxation_in_YBCO_superconductors`

**[R180]** HTS height drift under long-term disturbance, *J. Superconductivity and Novel Magnetism* (2019). *Why it matters:* the drift behaviour the HTS-plus-EMS trim must hold within 0.5 mm over 24 h (FAB-EXO-208). `link.springer.com/article/10.1007/s10948-019-05170-0`

**[R181]** HTS maglev test lines: the Chengdu (SWJTU) ring test line (ResearchGate), a science.gov topic page, Maglev-Cobra (*J. Phys. Conf. Ser.* 507, 032017; FAPESP), SupraTrans and SupraTrans II (IEEE; OSTI ETDE), and evico ZEK-DS. *Why it matters:* demonstrated HTS vehicles and the 5–8× area penalty relative to EMS (185 m² for 1,000 t) (FAB-EXO-113; EXOTIC §X3). `www.researchgate.net/publication/301579562_A_High-Temperature_Superconducting_Maglev_Ring_Test_Line_Developed_in_Chengdu_China`; `www.science.gov/topicpages/h/hts+maglev+test.html`; `iopscience.iop.org/article/10.1088/1742-6596/507/3/032017`; `revistapesquisa.fapesp.br/en/brazilian-magnetic-levitation-train-begins-new-phase-of-testing/`; `ieeexplore.ieee.org/document/1440126/`; `www.osti.gov/etdeweb/biblio/22001757`; `www.evico.de/en/products/zek-ds`

**[R182]** S-net for earthquake early warning, *Frontiers in Earth Science* (2021). *Why it matters:* offshore cabled observatories add seconds of warning for subduction events, the OffshoreCabled feed (FAB-EXO-801, FAB-EXO-804). `www.frontiersin.org/journals/earth-science/articles/10.3389/feart.2021.699439/full`

**[R183]** RTRI, ocean-bottom seismometer early warning for railways (13 s versus 10 s). *Why it matters:* measured lead-time gain from offshore sensing (FAB-EXO-804). `www.jstage.jst.go.jp/article/rtriqr/60/2/60_134/_pdf`

**[R184]** N-net offshore network: *Nature* (Japan news, 2023) and *Scientific American* (secondary). *Why it matters:* up to 20 s more earthquake warning and 20 min more tsunami warning, the ceiling for TIDEGATE timing (FAB-EXO-804, FAB-EXO-809). `www.nature.com/articles/d42473-023-00113-9`; `www.scientificamerican.com/article/japans-new-undersea-earthquake-detection-system-will-improve-tsunami/`

**[R185]** MOWLAS, Japan's integrated land and ocean seismic network, *Earth, Planets and Space* (2020). *Why it matters:* the network architecture behind a fused Network feed (FAB-EXO-802). `link.springer.com/article/10.1186/s40623-020-01250-x`

**[R186]** Y. Kodera et al., the PLUM method, *Bull. Seismol. Soc. Am.* 108(2), 983 (2018); D. Kilb et al., *J. Geophys. Res.* (2021). *Why it matters:* intensity-based warning that propagates observed shaking, the source of the 0.27027 s/km S-wave lead-time gain (FAB-EXO-803; EXOTIC §X10). `ui.adsabs.harvard.edu/abs/2018BuSSA.108..983K/abstract`; `agupubs.onlinelibrary.wiley.com/doi/full/10.1029/2020JB021053`

**[R187]** E. Lior et al., distributed acoustic sensing for earthquake early warning (2023), and dEPIC. *Why it matters:* fibre along a rail or coast as a Das feed; documented, research tier (FAB-EXO-801). `pmc.ncbi.nlm.nih.gov/articles/PMC9829724/`; `pmc.ncbi.nlm.nih.gov/articles/PMC12783127/`

**[R188]** Early-warning algorithms cited from memory in the research: Hoshiba (2011), Böse (FinDer), Chung (EPIC) and Tamaribuchi (IPF). Not fetched. *Why it matters:* the algorithm families that fused alerts come from; fabrail consumes their outputs and implements none of them.

**[R189]** Ground-motion and isolation sources cited from memory: J. W. Baker (2007) on pulse periods, Mavroeidis and Papageorgiou (2003) on near-fault pulses, Kalpakidis and Constantinou (2009) on lead-rubber bearing heating, and works by Hara, Ozawa and Akiyama. Not fetched. *Why it matters:* the pulse (Tp), fling and bearing-heating cases of the exotic scenarios (FAB-EXO-606, FAB-EXO-703; EXOTIC §X15).

**[R190]** G. Green, on the motion of waves in a variable canal of small depth and width, *Trans. Cambridge Philos. Soc.* (1838). Not fetched. *Why it matters:* Green's law, amplitude ∝ h^(−1/4), with long-wave speed √(g·h): 198 m/s at 4 km depth and ×4.47 from 4,000 m to 10 m (FAB-EXO-809; EXOTIC §X12).

**[R191]** Friedlander waveform (Wikipedia). *Why it matters:* the pressure–time form whose impulse fabrail checks in closed form, and the normal-reflection factor Pr/Ps (FAB-EXO-806; EXOTIC §X11). `en.wikipedia.org/wiki/Friedlander_waveform`

**[R192]** S. Glasstone and P. J. Dolan, *The Effects of Nuclear Weapons*, chapter 3. *Why it matters:* consulted only to confirm that blast arrival at kilometre ranges is seconds to tens of seconds. fabrail takes the arrival time t_a as an input, reports only the bound t_a ≤ R/c0, and reproduces none of the book's blast-scaling relations (FAB-EXO-807, FAB-EXO-808). `www.atomicarchive.com/resources/documents/effects/glasstone-dolan/chapter3.html`

**[R193]** G. F. Kinney and K. J. Graham, *Explosive Shocks in Air* (1985), and UFC 3-340-02. Not fetched. *Why it matters:* the standard air-blast references, deliberately not implemented (FAB-EXO-808).

**[R194]** Kantrowitz limit and tunnel boom (Wikipedia). *Why it matters:* the physics of pressure waves in tunnels, kept as a load case only after the tunnel-piston calculator was dropped (EXOTIC §X11). `en.wikipedia.org/wiki/Kantrowitz_limit`; `en.wikipedia.org/wiki/Tunnel_boom`

---

## Frontier tier (🔭)

Every item here is a what-if study (FRONTIER.md). Citing a source is not a claim that the capability is real.

**[R195]** A. Licciardi et al., instantaneous tracking of earthquake growth with elastogravity signals, *Nature* 606 (2022) (PEGSNet), with the PMC full text, an OSTI record and a Science News article (secondary). *Why it matters:* a deep-learning magnitude from prompt elasto-gravity signals for Mw ≥ 8.3 within tens of seconds: the PEGS magnitude advisory that may enter fusion but never trigger alone (FAB-EXO-810, FAB-FRN-003). `doi:10.1038/s41586-022-04672-7`; `pmc.ncbi.nlm.nih.gov/articles/PMC9177427/`; `www.osti.gov/pages/biblio/2310321-instantaneous-tracking-earthquake-growth-elastogravity-signals`; `www.sciencenews.org/article/machine-learning-gravity-earthquake-ai`

**[R196]** M. Vallée et al., observations and modelling of the elastogravity signals preceding direct seismic waves, *Science* 358 (2017); Vallée (2019) in *J. Geophys. Res.*; J.-P. Montagner et al. (2016) on the prompt gravity signal of Tohoku. *Why it matters:* the physics of PEGS arriving before the P wave, the "pressure wave extended" beat of the narrative (FAB-FRN-003; FRONTIER §F4). `doi:10.1126/science.aao0746`; `agupubs.onlinelibrary.wiley.com/doi/full/10.1029/2018JB017130`; `pmc.ncbi.nlm.nih.gov/articles/PMC5121411/`

**[R197]** K. Juhel et al., PEGS detection and modelling, *Geophys. J. Int.*, and Juhel et al. (2024) in *Communications Earth & Environment*. *Why it matters:* magnitude and mechanism about 2 min after origin, the realistic timescale of a PEGS advisory (FAB-FRN-003). `www.ipgp.fr/~jpm/PUBLICATIONS/19Juhel-GJI.pdf`; `doi:10.1038/s43247-024-01725-9`

**[R198]** Arias et al. (2025), PEGS for tsunami warning in Peru, *Geophys. J. Int.* 244(1), ggaf419; Arias et al. (2023) on Maule, *J. Geophys. Res.* *Why it matters:* PEGS magnitudes ≥ 8.2 at 300 s, the tsunami-warning use that TIDEGATE's advisory feed mirrors (FAB-FRN-003, FAB-EXO-810). `academic.oup.com/gji/article/244/1/ggaf419/8300379`; `agupubs.onlinelibrary.wiley.com/doi/abs/10.1029/2023JB026588`

**[R199]** T. Shimoda et al., early earthquake detection with a gravity gradiometer, *Geophys. J. Int.* 224(1), 533 (2021). *Why it matters:* the κ fit behind "Mw 7 at 140 km within 10 s" at 1e-15/√Hz, five orders beyond the demonstrated 1e-10/√Hz (FAB-FRN-003; FRONTIER §F4). `academic.oup.com/gji/article/224/1/533/5920617`

**[R200]** TOBA torsion-bar antenna: a review (arXiv:2412.01323) and a *Galaxies* 12(6), 78 article. *Why it matters:* the instrument path that would have to close the five-order sensitivity gap (FAB-FRN-003). `arXiv:2412.01323`; `doi:10.3390/galaxies12060078`

**[R201]** Next-generation gravity strainmeters: CHRONOS (arXiv:2604.05840), a torsion-pendulum detector by Ross et al. (arXiv:2101.01243), and infrasound noise on gravity detectors (arXiv:1801.04564). *Why it matters:* candidate instruments and the atmospheric noise floor they face, the "missing orders" of FAB-FRN-008. `arXiv:2604.05840`; `arXiv:2101.01243`; `arXiv:1801.04564`

**[R202]** M.-A. Meier et al., on the similar early growth of megathrust source time functions, *Science* (2017). Not fetched. *Why it matters:* no detector can know the final magnitude from the first seconds, which bounds every learned or gravity-based trigger (FRONTIER §F4).

**[R203]** S. Brûlé et al., experiments on seismic metamaterials: molding surface waves, *Phys. Rev. Lett.* 112, 133901 (2014), and a phys.org report (secondary). *Why it matters:* a field test with 5 m boreholes at 50 Hz, far above the 1 Hz where buildings respond, the anchor for the metabarrier mass law (FAB-FRN-004; FRONTIER §F5). `arXiv:1301.7642`; `phys.org/news/2014-04-field-possibility-deflecting-seismic-desired.html`

**[R204]** A. Colombi et al., forests as a natural seismic metamaterial, *Scientific Reports* 6, 19238 (2016). *Why it matters:* resonator arrays work at tens of hertz with tree-scale masses (FAB-FRN-004). `doi:10.1038/srep19238`

**[R205]** A. Palermo et al., engineered metabarrier as shield from seismic surface waves, *Scientific Reports* 6, 39356 (2016). *Why it matters:* the resonator-mass scaling ρ·v_s³/f³ that gives 1.8 Mt at 1 Hz and v_s = 100 m/s (FAB-FRN-004). `doi:10.1038/srep39356`

**[R206]** D. J. Colquitt et al., seismic metasurfaces (2017). *Why it matters:* the surface-wave theory behind the metabarrier what-if (FRONTIER §F5). `arXiv:1608.01792`

**[R207]** Seismic-metamaterial theory: seismic metamaterial (Wikipedia), bandgap estimation (arXiv:1612.03130), inerter-based metamaterials (arXiv:2206.03290), zero-frequency bandgaps (*J. Sound and Vibration*, 2025) and tunable-stiffness seismic metamaterials (*Acta Mechanica Sinica*, 2026). *Why it matters:* the inerter route is the only one that lowers the resonating mass, and none reaches building scale (FAB-FRN-004). `en.wikipedia.org/wiki/Seismic_metamaterial`; `arXiv:1612.03130`; `arXiv:2206.03290`; `www.sciencedirect.com/science/article/pii/S0022460X25001385`; `link.springer.com/article/10.1007/s10409-026-51240-x`

**[R208]** F. Matichard et al., seismic isolation of Advanced LIGO (2015). *Why it matters:* the best demonstrated active isolation, at instrument scale, the anchor tier for QZS and active-isolation what-ifs (FAB-FRN-005; FRONTIER §F6). `arXiv:1502.06300`

**[R209]** Room-temperature superconductor and LK-99 (Wikipedia, 2026). *Why it matters:* no ambient-pressure room-temperature superconductor exists (LK-99 is refuted), so the RTSC sweep is a what-if needing Tc ≈ 358 K (FAB-FRN-001; FRONTIER §F2). `en.wikipedia.org/wiki/Room-temperature_superconductor`; `en.wikipedia.org/wiki/LK-99`

**[R210]** F. Gömöry et al., experimental realization of a magnetic cloak, *Science* 335 (2012); a 3-D cloak working from DC to 250 kHz (arXiv:1511.08005). Not fetched (DOI only). *Why it matters:* lab-scale superconductor–ferromagnet cloaks, 🧪 at lab scale and 🔭 at structure scale (FAB-FRN-007; FRONTIER §F8). `doi:10.1126/science.1218316`; `arXiv:1511.08005`

**[R211]** C. Navau et al., long-distance transfer and routing of static magnetic fields, *Phys. Rev. Lett.* 112, 253901 (2014) (the magnetic hose), and the magnetic concentrator in *Phys. Rev. Lett.* 109, 263903 (2012). Publisher pages were not reachable; details from the arXiv version and memory. *Why it matters:* the concentrator gain bound ≤ B_sat/B0 of the cloak model (FAB-FRN-007). `arXiv:1304.6300`

**[R212]** Trapped-field records beyond bulk: 13.4 T near 5 K in tape stacks (arXiv:1709.03085) and a related trapped-field study (arXiv:1208.1854). *Why it matters:* more trapped field does not raise the pressure ceiling past rail saturation (FAB-FRN-002; FRONTIER §F3). `arXiv:1709.03085`; `arXiv:1208.1854`

**[R213]** T. Wolf et al., subpicotesla diamond magnetometry (2015); NV centre, superconducting gravimeter and distributed acoustic sensing (Wikipedia). *Why it matters:* the sensing technologies listed as documented-only in the frontier matrix (FRONTIER §F10). `arXiv:1411.6553`; `en.wikipedia.org/wiki/Nitrogen-vacancy_center`; `en.wikipedia.org/wiki/Superconducting_gravimeter`; `en.wikipedia.org/wiki/Distributed_acoustic_sensing`

**[R214]** NASA, FLOAT: Flexible Levitation on a Track (NIAC 2021 Phase I). *Why it matters:* diamagnetic graphite robots driven by flex-circuit thrust at > 0.5 m/s and ≤ 33 kg/m², the consistency check for the diamagnetic model and the SELENE anchor (FAB-FRN-009, FAB-FRN-006). `www.nasa.gov/general/float-flexible-levitation-on-a-track/`

**[R215]** Moonquake, lunar south pole and lunar regolith (Wikipedia). *Why it matters:* shallow moonquakes, permanently shadowed regions at 25–40 K and nanophase-iron dust that collects on permanent-magnet guideways, the SELENE caveats (FAB-FRN-006; FRONTIER §F7). `en.wikipedia.org/wiki/Moonquake`; `en.wikipedia.org/wiki/Lunar_south_pole`; `en.wikipedia.org/wiki/Lunar_regolith`
