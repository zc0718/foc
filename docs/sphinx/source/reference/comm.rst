_`comm`
=======

Three wire contracts, in increasing order of how much they assume. Each is a *format*: it
describes bytes and their meaning, and nothing here decides when to send them.

The three
---------

.. list-table::
   :header-rows: 1
   :widths: 26 44 30

   * - Header
     - What it defines
     - What it refuses to guess
   * - ``frame.hpp``
     - a frame layout, its status, and a checker that keeps sequence numbers honest
     - the transport: a frame is bytes, not a bus
   * - ``register_map.hpp``
     - the register table a host talks to, dense from 1 so enumeration has no holes
     - the encoding: registers are entries, not units
   * - ``encoder_stream.hpp``
     - the bit-level contract for an absolute encoder: layout, one frame, and the assembly
       across turns
     - the electrical layer: it reads a bit stream somebody else framed

Records that outlive the run
----------------------------

A frame is something you send; these two are something you **keep**. They share the rest of the
page's rule — bytes and their meaning, never a transport — but their reader arrives after a power
cycle, so what a decoder refuses matters more than what an encoder writes.

- ``commissioned_set.hpp`` — the numbers a commissioning run measures, in a form that survives a
  power cycle: what is stored, what is derived again on load, and the six ways decoding refuses
  (short buffer, foreign marker, unknown schema, a reserved byte that is not zero, a check value
  that does not match, a field that cannot be true).
- ``board_record.hpp`` — one production test's outcome in bytes: which unit, when, and every row it
  judged — so "this board passed" is a record with a payload rather than a log line.

Why a dense register map
------------------------

Register zero is not "the first register" in this library — it is the *absence* of one. The table
starts at 1 and has no holes, so enumeration is a loop and not a search, and a missing register
is a missing index rather than a reserved zero that every reader has to special-case.

Two rules that make a register map survivable
---------------------------------------------

- **Monotone**: a build never renumbers an existing register. New entries are appended, because a
  host that was flashed last quarter is still a host.
- **Self-describing enough to be checked**: every entry says what it is, so a host reading a
  register it does not know can say so instead of using the value.

Sequence numbers
----------------

``frame.hpp``'s checker exists because the two common transport failures — a dropped frame and a
repeated one — look the same to a receiver that only reads payloads. A sequence number makes them
different: a gap is a loss, a repeat is a duplicate, and both are answers the receiver can act on
instead of a value it half-trusts.

What is out of scope
--------------------

- **No bus implementation.** CAN, EtherCAT, UART: the frames fit any of them, and no page here
  promises a timing guarantee on a bus.
- **No security layer.** Authentication and integrity are transport concerns; this layer checks
  what it can check locally (sequence, status) and says so.

Related
-------

- the C face a host most often talks to: :doc:`c-api`
- the byte-level seams of the loop: :doc:`core`'s ``trace.hpp`` and ``trace_parse.hpp``
