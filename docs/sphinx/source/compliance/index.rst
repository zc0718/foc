_`What may be claimed, and to whom`
===================================

This page answers one question: **which sentence about compliance may this project put in front
of which customer today?** It does not answer "what would we have to buy to say more" — that is
:doc:`tier`, and it does not answer "what does my industry ask first" — that is
:doc:`industries`.

Four readers, four pages
------------------------

.. toctree::
   :maxdepth: 1
   :hidden:

   tier
   industries
   language

.. list-table::
   :header-rows: 1
   :widths: 24 26 50

   * - Who is asking
     - Go to
     - What that page gives
   * - Procurement, or a customer's buyer
     - :doc:`tier`
     - the five claim tiers: the sentence each one buys, the documents it needs, and where this
       project stands in each
   * - A safety engineer in one industry
     - :doc:`industries`
     - eight cards, one per industry: what they ask, what they actually read, what can be given
       today, and what is missing
   * - A tool or language reviewer
     - :doc:`language`
     - the language claim in detail: the rule table, its two lists, the deviation permits, and
       what each of them can and cannot support
   * - An auditor
     - this page, then :doc:`../boundaries`
     - the four sentences below, and the list of things this library does not do at all
   * - A customer's licence reviewer
     - :doc:`../third-party`
     - every dependency we link with its licence and the paths it may appear on, and the licence of
       each upstream project this library is allowed to read from

The four sentences this project never says
------------------------------------------

These are fixed. They are not softened by a modifier, and no page on this site may contradict
them. The wording is generated from `.github/project/compliance/claims.json`, so the API
documentation and this site say the same four things.

.. list-table::
   :header-rows: 1
   :widths: 18 82

   * - Never
     - The sentence, and why
   * - **Level**
     - This library claims no ASIL, SIL or Class B level. Levels come out of a hazard analysis
       and safety goals for a **whole machine**; a library has no such context, so it can supply
       evidence about its own behaviour and nothing about the level of the system it lands in.
   * - **Rule set**
     - This library claims no conformance to AUTOSAR C++14: it is built as C++17, and the
       document is a C++14 rule set. What can be said is tighter and checkable: the rules are
       organised by that document's numbering, and every C++17 delta is recorded.
   * - **Third party**
     - NASA and JPL coding standards are not a compliance basis here; their use is a free,
       third-party-readable checklist.
   * - **Independence**
     - This library claims reproducibility, not independent verification: a third party can
       re-run every gate and get the same numbers, which is not the same as having been verified
       independently.

Two of the four are worth reading twice. "Reproducible" is a property of the artefact and can be
demonstrated in an afternoon; "independently verified" is a property of the *relationship*
between two organisations and cannot be demonstrated by the first organisation at all. And a rule
set is a document about a language version, not a certificate about this code.

Where the evidence is
---------------------

Every claim above is re-runnable, and a claim that cannot be re-run is not evidence. These are the
artefacts, and the command that regenerates each:

.. list-table::
   :header-rows: 1
   :widths: 42 58

   * - Artefact
     - Re-run with
   * - the rule table, one rule per line
     - ``python3 .github/project/compliance/gen_subset.py --check`` (stale means exit 1)
   * - the four counts, per language
     - ``python3 .github/project/compliance/subset_gate.py --summary``
   * - the deviation permits, five elements each
     - ``python3 .github/project/gates.py subset`` (also checks the NOLINT bijection)
   * - this project's own strictness, composed from outside the pinned file
     - ``python3 .github/project/gates.py enforce``
   * - the evidence level of each capability
     - the claim table in the project ``README.md``, whose rows carry an evidence level and a
       test case

No count is printed on this page on purpose: a number copied into prose is a second source of
truth with a shelf life. The counts live in the artefacts above.

Related
-------

- what this library does not do at all: :doc:`../boundaries`
- what each capability owes before it may be claimed: :doc:`../capabilities`
- how the evidence is produced: :doc:`../usage`
