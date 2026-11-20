document.querySelectorAll("[data-copy-target]").forEach((button) => {
  button.addEventListener("click", async () => {
    const target = document.getElementById(button.dataset.copyTarget);
    const label = button.querySelector(".copy-label");
    if (!target || !label) return;

    const text = target.textContent.trim();
    if (navigator.clipboard && window.isSecureContext) {
      try {
        await navigator.clipboard.writeText(text);
        label.textContent = "Copied";
        return;
      } catch {
        // Local file previews may not grant clipboard permission.
      }
    }

    const selection = window.getSelection();
    const range = document.createRange();
    range.selectNodeContents(target);
    selection.removeAllRanges();
    selection.addRange(range);
    let copied = false;
    try {
      copied = document.execCommand("copy");
    } catch {
      // Keep the citation selected for manual copying.
    }
    if (copied) {
      selection.removeAllRanges();
      label.textContent = "Copied";
    } else {
      label.textContent = "Selected — press Ctrl+C";
    }
  });
});
