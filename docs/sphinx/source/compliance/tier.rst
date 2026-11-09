_`Claim tiers`
==============

A tier is one sentence this project may say, plus the documents that sentence needs. The tiers
exist so that "where are we?" has one answer inside the project and one answer in front of a
customer, and so that the answer cannot be upgraded by enthusiasm.

The five tiers
--------------

.. list-table::
   :header-rows: 1
   :widths: 8 40 30 22

   * - Tier
     - The sentence it buys
     - What that sentence needs
     - Where this project stands
   * - **T0**
     - "The language subset is organised by a downloadable draft or guideline; rule numbers come
       from the toolchain."
     - no paid document at all
     - **holds**: the free material is on disk, each copy with a sha256
   * - **T1**
     - "Organised by rule number: N implemented, M not applicable, K deviated, deviations
       submitted as permits."
     - the standard itself, plus the compliance guidance that defines the four categories
     - **holds**, with one exception: the mapping from a tool check to a rule number does not
       exist yet, so N/M/K cannot be filled in. That is the gap between T0 and T1, and it is
       named as an open item rather than rounded up
   * - **T2**
     - the T1 sentence plus **quantitative hardware failure evidence**.
     - failure-rate data for the components, which lives with the hardware supplier, not here
     - **half**: the method can be cited; no number can be computed by this repository, and the
       component data is not ours to publish
   * - **T3**
     - "assessed by a third party."
     - an assessment contract
     - **not this library's to issue**: it can produce evidence, it cannot produce independence
   * - **T4**
     - measured on real hardware, with the condition recorded.
     - a motor, an inverter, a current probe
     - **not done**: there is no hardware in the loop, and :doc:`../boundaries` says so item by
       item

Two of those rows are limits, not gaps. T3 needs a second organisation and T4 needs a bench;
neither is bought, scheduled or wished into existence, and a page that showed them as "in
progress" would be lying about what a library can be.

Two rules that keep the table honest
------------------------------------

1. **The status column is written from an actual result, never from intent.** If a row says the
   document is in hand, there is a copy on disk with a recorded sha256, and the entry names the
   edition that was checked against its cover and contents page.
2. **A tier is never upgraded in conversation.** To move a row up, point its "needs" column at an
   actual copy — a file name plus the sha256 recorded when it was fetched. A row whose needs
   cannot be pointed at is **T0**, whatever anyone remembers.

The reason for rule 2 is arithmetic, not distrust: the failure mode of a claim ladder is that
everyone remembers the highest rung anyone ever mentioned, and the numbers behind it were never
written down.

Which tier lets you discuss which standard
------------------------------------------

The tiers are not industry-specific; the standards are. This is the join between the two pages,
and :doc:`industries` fills in the industry side:

.. list-table::
   :header-rows: 1
   :widths: 20 34 46

   * - Standard family
     - Becomes discussable at
     - Because
   * - MISRA C++ / compliance guidance
     - **T1**
     - the four categories and the five elements of a deviation permit are what T1 defines
   * - ISO 26262 (SEooC shape), IEC 61508-3 (technique tables)
     - **T1**
     - both are "organised by clause number" claims, plus the safety manual shape
   * - ISO 26262 tool confidence, IEC 61508 tool qualification
     - **T1**, as an argument rather than a certificate
     - the evidence is a self-test per gate, a differential build with two compilers, and a
       known-defect list — none of which is a formal tool qualification
   * - any ASIL, SIL, PL, Class, DAL or MDD level
     - **never**
     - levels belong to a whole machine; see the four sentences on :doc:`index`
   * - "third-party assessed"
     - **T3**
     - which this library does not issue

Related
-------

- the four sentences, and the evidence behind each: :doc:`index`
- the industry-by-industry answer: :doc:`industries`
- what the language claim can support in detail: :doc:`language`
- what the library does not do at all: :doc:`../boundaries`
