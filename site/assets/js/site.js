document.documentElement.classList.add("js");

const copyButton = document.querySelector("[data-copy-target]");

if (copyButton) {
  copyButton.addEventListener("click", async () => {
    const target = document.getElementById(copyButton.dataset.copyTarget);
    const label = copyButton.querySelector(".copy-label");

    if (!target || !label) return;

    try {
      await navigator.clipboard.writeText(target.textContent.trim());
      label.textContent = "Copied";
      window.setTimeout(() => {
        label.textContent = "Copy citation";
      }, 1800);
    } catch {
      const selection = window.getSelection();
      const range = document.createRange();
      range.selectNodeContents(target);
      selection.removeAllRanges();
      selection.addRange(range);
      label.textContent = "Selected — press Ctrl+C";
    }
  });
}

const year = document.getElementById("copyright-year");
if (year) year.textContent = String(new Date().getFullYear());
