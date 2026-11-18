// Ghidra headless script: Cython-aware annotation of prtouch_v1_wrapper.
// args: <methods.tsv> <strtab.tsv> <shift-hex>
//  - marks .got read-only so MIPS PIC GOT loads fold into globals
//  - labels Cython string-table slots (n_<name>, s_<text>) shifted by <shift>
//  - creates + names method entry functions (M_<name>) shifted by <shift>
import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.mem.MemoryBlock;
import ghidra.program.model.symbol.SourceType;
import java.nio.file.*;

public class Annotate extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] a = getScriptArgs();
        long shift = Long.parseLong(a[2], 16);
        for (MemoryBlock b : currentProgram.getMemory().getBlocks()) {
            if (b.getName().equals(".got")) { b.setWrite(false); println("got ro " + b.getStart()); }
        }
        for (String line : Files.readAllLines(Paths.get(a[1]))) {
            String[] p = line.split("\t", 3);
            if (p.length < 3) continue;
            String s = p[2].replaceAll("[^A-Za-z0-9_]", "_");
            if (s.length() > 60) s = s.substring(0, 60);
            String lbl = (p[1].equals("n") ? "n_" : "s_") + s;
            Address ad = toAddr(Long.parseLong(p[0], 16) + shift);
            try { createLabel(ad, lbl, true, SourceType.USER_DEFINED); } catch (Exception e) { println("lbl fail " + lbl); }
        }
        for (String line : Files.readAllLines(Paths.get(a[0]))) {
            String[] p = line.split("\t");
            if (p.length < 2 || (p[1].startsWith("__") && !p[1].equals("__init__")) || p[1].startsWith("func_")) continue;
            Address ad = toAddr(Long.parseLong(p[0], 16) + shift);
            disassemble(ad);
            Function f = getFunctionAt(ad);
            if (f == null) {
                Function c = getFunctionContaining(ad);
                if (c != null) removeFunction(c);
                f = createFunction(ad, "M_" + p[1]);
            } else f.setName("M_" + p[1], SourceType.USER_DEFINED);
            if (f == null) println("could not create " + p[1] + " at " + ad);
        }
        analyzeChanges(currentProgram);
    }
}
