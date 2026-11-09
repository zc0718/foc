_`By industry`
==============

Eight industries, eight cards, the same six questions in the same order: what the customer asks,
what they actually read, what can be given today, what is missing, what may be said, and what may
not. The order is the order the questions arrive in, which is why the answer to "can you do my
industry" is on the fifth line and not the first.

The standard numbers below carry the edition that was checked against an official catalogue. Where
an edition is named, it is the current one at the time of writing and the previous edition is
named where it matters — a superseded number quoted with a straight face is the fastest way to
lose a technical audience.

Automotive — ISO 26262 (SEooC shape)
------------------------------------

**Asked:** "Do you conform to MISRA C++, and at which ASIL?"

**They read:** the four counts of the rule table (implemented / not applicable / deviated / open),
the deviation permits, a compliance summary, and the SEooC delivery package.

**Today:** the language baseline is enforced per public header; the rule table and the permits
exist and are gated; the SEooC *shape* is what the delivery package follows.

**Missing:** the mapping from a tool check to a rule number, so the three counts that a reader
actually looks for cannot be filled in yet. Until then the sentences exist and the numbers do not.

**May say:** "organised by MISRA C++:2023 rule numbers, deviations submitted as permits with a
compliance summary; delivered in SEooC shape."

**May not say:** "conforms to ISO 26262"; any ASIL.

Industrial — IEC 61508-3:2010
-----------------------------

**Asked:** "Which SIL, and how is the technique table filled in?"

**They read:** the HR / R / NR ratings in the technique tables, and structural coverage evidence.

**Today:** the annexes that define those ratings are in hand (the technique tables and the
explanatory layer that says what each technique name actually demands), and this repository
produces the two evidence shapes those rows ask for: a per-decision coverage gate and a mutation
baseline.

**Missing:** the row-by-row mapping from those tables to this repository's gates.

**May say:** "organised by the IEC 61508-3 technique tables (no SIL claimed); the safety manual is
organised per the normative safety-manual annex."

**May not say:** any SIL 1–3; "conforms to IEC 61508".

Appliance — IEC 60730-1:2022, Annex H (Class B)
-----------------------------------------------

**Asked:** "Which Class B self-tests are implemented?"

**They read:** the Annex H self-test list — CPU registers, program counter, ROM and RAM integrity,
clock — and where the periodic self-test is called from.

**Today:** a board check that answers a different question: did this drive get wired and sensed
correctly. Eight items, with a diagnosis that names one cause.

**Missing:** **this is a real gap, not a boundary.** Annex H asks whether *this MCU* is still
healthy; the board check asks whether *this drive* is connected correctly. Nothing in this library
tests the CPU, ROM, RAM or clock — those belong to the MCU startup code, and integrators usually
have them. What this library can do is state the boundary; it cannot present its eight items as
Annex H evidence.

**May say:** "the drive-level self-check covers wiring, sensing and parameters; MCU-level CPU,
ROM, RAM and clock self-tests are outside this library and are expected from the integrator's
startup code."

**May not say:** "supports Class B"; anything that presents the board check as Annex H evidence.

Machinery — ISO 13849-1:2023
----------------------------

**Asked:** "Can you reach PL d?"

**They read:** the architecture category, MTTFd, diagnostic coverage, common-cause failure — and
the calculation that combines them.

**Today:** nothing, and that is the answer. This library has no gate authority, no safety input
and no two-channel architecture, so every input the performance-level argument needs lives with
somebody else.

**Missing:** nothing. A PL argument for the machine is the integrator's safety function.

**May say:** "the safety function of the machine belongs to the integrator; this library supplies
the control law and its own evidence, and takes no part in the PL calculation."

**May not say:** any performance level or category.

Note the edition: ISO 13849-1:2015 was withdrawn when the 2023 edition was published. A proposal
quoting 2015 is quoting a superseded document.

Robotics — ISO 10218-1:2025 / -2:2025
-------------------------------------

**Asked:** "Which standard does a joint module fall under?"

**They read:** the same machinery answer (ultimately ISO 13849 or IEC 62061), plus the
collaborative-operation requirements.

**Today:** nothing beyond the machinery answer — and one thing worth having: the current numbers
and editions, so that a conversation starts from `ISO 10218-1:2025` rather than from a remembered
year.

**Missing:** nothing that belongs here. The value of this row is that the numbers are right, not
that a claim can be made.

**May say:** "the safety function belongs to the machine; where it lands is the integrator's
choice — commonly ISO 13849-1, IEC 62061 or ISO 10218-1."

**May not say:** any safety level.

Two edition facts a reader should know before quoting this industry: the 2011 editions of
ISO 10218-1 and -2 were replaced by the 2025 editions, and most of ISO/TS 15066:2016 (collaborative
robots — the biomechanical limits and the four operating methods) moved into ISO 10218-2:2025.

Rail — EN 50716:2023
--------------------

**Asked:** "Which EN 50128 integrity level?"

**They read:** software integrity levels, the required independence of verification and
validation, and tool qualification.

**Today:** reproducibility, and only reproducibility. A third party who clones the repository can
re-run every gate, recompute every sha256 and replay every case, and get the same numbers.

**Missing:** **independence.** Reproducibility is a property of an artefact; independence is a
property of the relationship between two organisations. SIL 1 and above want independent
verification and validation, which one author with tooling cannot supply at any price. This is one
of the three gaps that money cannot close.

**May say:** "the evidence is reproducible; independent verification has not been performed and
must be supplied by the purchaser or a third party."

**May not say:** any rail integrity level; or "independently verified" as a rendering of
"reproducible".

One number matters here and it changed recently: **EN 50716:2023** supersedes EN 50128:2011 (with
its amendments) and EN 50657:2017 for railway software. A customer who opens with "EN 50128" is
using the former name, and answering in that name without saying so is the wrong kind of
agreement.

Medical — IEC 62304:2006+A1:2015
--------------------------------

**Asked:** "Which 62304 safety class?"

**They read:** the material that justifies the class, and the device maker's own risk file under
ISO 14971:2019.

**Today:** the same rule table and the same gates as the automotive and industrial rows — they are
not industry-specific.

**Missing:** the class and the risk file. Class A/B/C follows from what the software can do to a
patient, which is a property of the device; ISO 14971 risk management belongs to the device maker.

**May say:** "the software item is organised by the shared rule table; safety classification and
risk management belong to the device manufacturer."

**May not say:** any 62304 class; or anything that implies a risk file exists here.

Aerospace — DO-178C — **not pursued**
-------------------------------------

**Asked:** "Which DAL?"

**They read:** the DAL allocation, MC/DC for DAL A, and a complete lifecycle data package.

**Today:** nothing, and this is a decision rather than a queue.

**Missing:** it conflicts with this project's foundation. DO-178C constrains C++ use (typically a
restricted subset plus object-code-to-source traceability), and a DAL argument wants hardware in
the loop. Neither is available here, and adding them is not a purchase.

**May say:** nothing. This industry is out of scope, in writing.

**May not say:** any DAL; and no page here may use a coding standard from this world to imply that
the path is being walked.

Three gaps that money cannot close
----------------------------------

These belong in a delivery note rather than in a brochure, and none of them is "we have not bought
it yet".

**Independent verification and validation.** One author plus tooling can produce reproducibility.
It cannot produce independence. Say the first, never the second.

**Tool confidence.** Compilers, linters, coverage tools and mutation tools here are not formally
qualified, and buying a qualification package would not help: what is missing is the argument.
Three free evidence shapes exist — a self-test per gate (each new gate must prove it goes red when
its evidence is removed), a differential build of the same header with two compilers, and a
known-defect list per tool. Together they are the skeleton of a tool-confidence argument; they are
not a certificate.

**Hardware evidence.** There is no board in the loop, so the bottom layer of any safety argument is
missing, and compliance paperwork cannot fill it. Only a bench can.

Related
-------

- the tiers these answers are graded by: :doc:`tier`
- the four sentences no industry changes: :doc:`index`
- what the library does not do at all: :doc:`../boundaries`
- what each capability owes: :doc:`../capabilities`
