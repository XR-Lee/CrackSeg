const dialog = document.querySelector('.image-dialog');
let priorFocus;
document.querySelectorAll('[data-zoom]').forEach(button => button.addEventListener('click', () => {
  const source = button.querySelector('img');
  priorFocus = button;
  const expanded = dialog.querySelector('img');
  expanded.src = source.src;
  expanded.alt = source.alt;
  document.getElementById('image-dialog-title').textContent = button.dataset.zoom;
  dialog.showModal();
  dialog.querySelector('.dialog-scroll').scrollTo({top:0,left:0});
  document.body.style.overflow = 'hidden';
}));
dialog.querySelector('.dialog-close').addEventListener('click', () => dialog.close());
dialog.addEventListener('click', event => { if (event.target === dialog) dialog.close(); });
dialog.addEventListener('close', () => {document.body.style.overflow = ''; if (priorFocus) priorFocus.focus();});
const copyButton = document.getElementById('copy-citation');
if (copyButton) copyButton.addEventListener('click', async () => {
  const text = document.getElementById('bibtex').textContent;
  const status = document.getElementById('copy-status');
  if (navigator.clipboard && window.isSecureContext) {
    try {
      await navigator.clipboard.writeText(text);
      status.textContent = 'Citation copied.';
      return;
    } catch {
      // Some browsers deny the Clipboard API for local files.
    }
  }
  const field = document.createElement('textarea');
  field.value = text;
  field.readOnly = true;
  field.style.position = 'fixed';
  field.style.opacity = '0';
  document.body.appendChild(field);
  try {
    field.select();
    status.textContent = document.execCommand('copy')
      ? 'Citation copied.' : 'Select and copy the citation above.';
  } catch {
    status.textContent = 'Select and copy the citation above.';
  } finally {
    field.remove();
    copyButton.focus();
  }
});
