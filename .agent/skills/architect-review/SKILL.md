---
name: architect-review
description: Performs a comprehensive, architect-level code review focusing on data flow integrity, concurrency, and safety.
---

# Architect Review Skill

This skill enables the agent to perform a deep-dive code review simulating a Senior Embedded Architect.

## When to Use
Use this skill when the user asks for a "review," "audit," "check," or "verify" of a specific feature, module, or data path (e.g., "Review the CAN fault handling" or "Check the FOC current loop").

## Instructions

1.  **Identify the Scope:**
    *   Determine the start point (Source) and end point (Sink) of the data/logic flow requested.
    *   Identify relevant files (Drivers, Middleware, Application).

2.  **Trace the Path:**
    *   Read the relevant files using `view_file`.
    *   Mentally map the call stack and data flow.
    *   *Self-Correction:* If a function call jumps to a file you haven't read, read that file too.

3.  **Audit Criteria (The "Rule"):**
    *   **Link Integrity:** verify data isn't lost or ignored between layers.
    *   **Concurrency:** Check for `volatile` on shared globals and critical sections ( `ENTER_CRITICAL()` ) for atomicity.
    *   **Types & Units:** Verify no implicit casting issues or unit mismatches.
    *   **Error Handling:** Ensure return values are checked.

4.  **Report Generation:**
    *   Output the review in the format specified below.

## Output Format

```markdown
### 🛡️ Architect Code Review: [Feature Name]

#### 🔍 Link Trace
`ISR() -> Layer_Process() -> App_Logic()` (Visual flow)

#### ✅ Good Practices
- [Item 1]
- [Item 2]

#### ⚠️ Critical Risks (Must Fix)
- **Race Condition:** `global_var` accessed in `file.c:20` (ISR) and `file.c:50` (Main) without protection.
- **Data Loss:** Return value of `HAL_CAN_Transmit` ignored in `can.c:100`.

#### 🛠️ Recommendations
- Extract magic number `0x123` to `MACRO_NAME`.
- Change type of `variable` to `uint32_t` to prevent overflow.

#### 🏁 Verdict
[READY / NEEDS WORK]
```
