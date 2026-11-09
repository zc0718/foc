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
