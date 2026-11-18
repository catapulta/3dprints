// Ghidra headless post-script: write each function's decompiled C to <outdir>/funcs/<name>.c
// and an index (address, size, name) to <outdir>/index.tsv.
import ghidra.app.script.GhidraScript;
import ghidra.app.decompiler.*;
import ghidra.program.model.listing.*;
import java.io.*;

public class DumpDecomp extends GhidraScript {
    @Override
    public void run() throws Exception {
        String outDir = getScriptArgs()[0];
        File funcs = new File(outDir, "funcs");
        funcs.mkdirs();
        DecompInterface di = new DecompInterface();
        di.openProgram(currentProgram);
        PrintWriter idx = new PrintWriter(new FileWriter(new File(outDir, "index.tsv")));
        FunctionIterator it = currentProgram.getFunctionManager().getFunctions(true);
        while (it.hasNext() && !monitor.isCancelled()) {
            Function f = it.next();
            if (f.isExternal() || f.isThunk()) continue;
            String name = f.getName().replaceAll("[^A-Za-z0-9_.]", "_");
            idx.println(f.getEntryPoint() + "\t" + f.getBody().getNumAddresses() + "\t" + f.getName());
            DecompileResults r = di.decompileFunction(f, 300, monitor);
            if (r == null || r.getDecompiledFunction() == null) continue;
            String fn = name.length() > 150 ? name.substring(0, 150) : name;
            try (PrintWriter w = new PrintWriter(new FileWriter(new File(funcs, fn + "@" + f.getEntryPoint() + ".c")))) {
                w.print(r.getDecompiledFunction().getC());
            }
        }
        idx.close();
    }
}
